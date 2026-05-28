import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import matplotlib.pyplot as plt
import matplotlib
import matplotlib.patches as mpatches
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import os
import threading
import time
import re
import serial
import serial.tools.list_ports

matplotlib.rcParams['font.family'] = ['Yu Gothic', 'MS Gothic', 'Meiryo', 'sans-serif']

from dxf_reader import read_dxf
from gcode_generator import generate_gcode
from kerf_offset import compute_offset_points, compute_path_types

MACHINE_W = 1200
MACHINE_H = 800
COLORS = ['#2196F3', '#4CAF50', '#FF5722', '#9C27B0', '#FF9800']


class PlasmaCamApp:
    def __init__(self, root):
        self.root = root
        self.root.title('Plasma CAM - GRBL')
        self.root.geometry('1450x860')

        # dxf_entries: list of {'name', 'paths', 'offset_x', 'offset_y'}
        self.dxf_entries = []
        self._all_artists = []   # list of dicts per (entry, path)
        self._legend_text = None

        self.ser = None
        self.streaming = False
        self.polling = False
        self.torch_x = 0.0
        self.torch_y = 0.0
        self.serial_lock = threading.Lock()

        # drag state
        self._drag_mode = None
        self._drag_start_px = None
        self._drag_transform = None
        self._drag_xlim = None
        self._drag_ylim = None
        self._drag_offset_start = None
        self._drag_entry_idx = None

        self._build_menu()
        self._build_ui()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    # ------------------------------------------------------------------ menu
    def _build_menu(self):
        mb = tk.Menu(self.root)
        fm = tk.Menu(mb, tearoff=0)
        fm.add_command(label='DXFを追加', command=self.open_dxf)
        fm.add_command(label='Gコードを保存', command=self.save_gcode)
        fm.add_separator()
        fm.add_command(label='終了', command=self._on_close)
        mb.add_cascade(label='ファイル', menu=fm)
        self.root.config(menu=mb)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        main = ttk.Frame(self.root)
        main.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        cf = ttk.LabelFrame(main, text='マシンビュー / DXFプレビュー')
        cf.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig, self.ax = plt.subplots(figsize=(9, 7))
        self.fig.subplots_adjust(bottom=0.05, top=0.95)
        self.canvas_widget = FigureCanvasTkAgg(self.fig, master=cf)
        self.canvas_widget.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        hint = ttk.Label(cf,
                         text='🖱 左ドラッグ: 選択ファイルを移動   右ドラッグ: 視点移動   ホイール: ズーム',
                         foreground='#555')
        hint.pack(pady=(0, 3))

        self._init_canvas()
        self._connect_canvas_events()

        right = ttk.Frame(main, width=300)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(5, 0))
        right.pack_propagate(False)

        nb = ttk.Notebook(right)
        nb.pack(fill=tk.BOTH, expand=True)

        t1 = ttk.Frame(nb)
        nb.add(t1, text='  設定  ')
        t2 = ttk.Frame(nb)
        nb.add(t2, text='  機械制御  ')

        self._build_settings_tab(t1)
        self._build_control_tab(t2)

    # ------------------------------------------------------------------ canvas
    def _init_canvas(self):
        self.ax.clear()
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)

        rect = mpatches.Rectangle(
            (0, 0), MACHINE_W, MACHINE_H,
            linewidth=2, edgecolor='#607D8B', facecolor='#FAFAFA',
            linestyle='--', zorder=0
        )
        self.ax.add_patch(rect)
        self.ax.text(MACHINE_W / 2, MACHINE_H + 20,
                     f'マシンサイズ  {MACHINE_W} × {MACHINE_H} mm',
                     ha='center', fontsize=9, color='#607D8B')

        for x, y, label in [(0, 0, '(0,0)'), (MACHINE_W, 0, f'({MACHINE_W},0)'),
                             (0, MACHINE_H, f'(0,{MACHINE_H})'),
                             (MACHINE_W, MACHINE_H, f'({MACHINE_W},{MACHINE_H})')]:
            self.ax.text(x, y, label, fontsize=7, color='#90A4AE',
                         ha='center', va='center')

        self.torch_cross, = self.ax.plot([0], [0], 'r+',
                                         markersize=22, markeredgewidth=2.5, zorder=10)
        self.torch_dot, = self.ax.plot([0], [0], 'ro', markersize=7, zorder=11)

        self.ax.set_xlim(-80, MACHINE_W + 80)
        self.ax.set_ylim(-60, MACHINE_H + 60)
        self.ax.set_title(f'マシン: {MACHINE_W} × {MACHINE_H} mm  |  トーチ: (0.000, 0.000)')
        self.canvas_widget.draw()

    def _connect_canvas_events(self):
        cw = self.canvas_widget
        cw.mpl_connect('scroll_event',         self._on_scroll)
        cw.mpl_connect('button_press_event',   self._on_press)
        cw.mpl_connect('button_release_event', self._on_release)
        cw.mpl_connect('motion_notify_event',  self._on_motion)

    def _on_scroll(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        factor = 0.8 if event.button == 'up' else 1.25
        cx, cy = event.xdata, event.ydata
        self.ax.set_xlim([cx + (x - cx) * factor for x in self.ax.get_xlim()])
        self.ax.set_ylim([cy + (y - cy) * factor for y in self.ax.get_ylim()])
        self.canvas_widget.draw_idle()

    def _on_press(self, event):
        if event.inaxes != self.ax:
            return
        self._drag_start_px = (event.x, event.y)
        self._drag_transform = self.ax.transData.inverted().frozen()
        if event.button == 1:
            self._drag_mode = 'dxf'
            idx = self._selected_entry_idx()
            if idx is not None:
                e = self.dxf_entries[idx]
                self._drag_offset_start = (e['offset_x'], e['offset_y'])
                self._drag_entry_idx = idx
            else:
                self._drag_offset_start = (0.0, 0.0)
                self._drag_entry_idx = None
        elif event.button == 3:
            self._drag_mode = 'pan'
            self._drag_xlim = list(self.ax.get_xlim())
            self._drag_ylim = list(self.ax.get_ylim())

    def _on_release(self, event):
        self._drag_mode = None
        self._drag_start_px = None

    def _on_motion(self, event):
        if self._drag_mode is None or self._drag_start_px is None:
            return
        dpx = event.x - self._drag_start_px[0]
        dpy = event.y - self._drag_start_px[1]
        p0 = self._drag_transform.transform([0, 0])
        p1 = self._drag_transform.transform([dpx, dpy])
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]

        if self._drag_mode == 'dxf' and self._drag_entry_idx is not None:
            ox = self._drag_offset_start[0] + dx
            oy = self._drag_offset_start[1] + dy
            e = self.dxf_entries[self._drag_entry_idx]
            e['offset_x'] = ox
            e['offset_y'] = oy
            self.offset_x.set(f'{ox:.1f}')
            self.offset_y.set(f'{oy:.1f}')
            self._update_entry_display(self._drag_entry_idx)
        elif self._drag_mode == 'pan':
            self.ax.set_xlim([v - dx for v in self._drag_xlim])
            self.ax.set_ylim([v - dy for v in self._drag_ylim])
            self.canvas_widget.draw_idle()

    def _update_entry_display(self, entry_idx):
        """Fast redraw for a single entry during drag."""
        if entry_idx >= len(self.dxf_entries):
            return
        e = self.dxf_entries[entry_idx]
        ox, oy = e['offset_x'], e['offset_y']

        for art in self._all_artists:
            if art['entry_idx'] != entry_idx or art['line'] is None:
                continue
            pts = art['path'].get_display_points()
            if not pts:
                continue
            xs = [p[0] + ox for p in pts]
            ys = [p[1] + oy for p in pts]
            art['line'].set_xdata(xs)
            art['line'].set_ydata(ys)

            off_pts = art['offset_pts']
            if art['offset_line'] is not None and off_pts:
                oxs = [p[0] + ox for p in off_pts]
                oys = [p[1] + oy for p in off_pts]
                art['offset_line'].set_xdata(oxs)
                art['offset_line'].set_ydata(oys)
                sx, sy = oxs[0], oys[0]
            else:
                sx, sy = xs[0], ys[0]

            art['marker'].set_xdata([sx])
            art['marker'].set_ydata([sy])
            if art['label'] is not None:
                art['label'].set_position((sx, sy))

        self.canvas_widget.draw_idle()

    def _draw_paths(self):
        for art in self._all_artists:
            for key in ('line', 'offset_line', 'marker', 'label'):
                a = art.get(key)
                if a is not None:
                    try:
                        a.remove()
                    except Exception:
                        pass
        self._all_artists = []

        if self._legend_text is not None:
            try:
                self._legend_text.remove()
            except Exception:
                pass
            self._legend_text = None

        if not hasattr(self, 'torch_cross'):
            self._init_canvas()

        try:
            kerf = float(self.kerf_width.get())
        except (ValueError, AttributeError):
            kerf = 0.0

        multi = len(self.dxf_entries) > 1
        legend_lines = ['切断順  (●開始点)']
        path_global_idx = 0

        for entry_idx, entry in enumerate(self.dxf_entries):
            ox, oy = entry['offset_x'], entry['offset_y']
            paths = entry['paths']
            inner_flags = compute_path_types(paths)

            for j, path in enumerate(paths):
                pts = path.get_display_points()
                is_inner = inner_flags[j]
                ptype = '内側(穴)' if is_inner else ('外側' if path.closed else '開路')
                color = COLORS[path_global_idx % len(COLORS)]

                off_pts = None
                if kerf > 0 and path.closed:
                    off_pts = compute_offset_points(path, kerf, is_inner=is_inner)

                prefix = f'[{entry["name"][:10]}] ' if multi else ''
                legend_lines.append(f'  {path_global_idx + 1}: {prefix}{ptype}')

                art = {
                    'entry_idx': entry_idx,
                    'path': path,
                    'offset_pts': off_pts,
                    'line': None, 'offset_line': None,
                    'marker': None, 'label': None,
                }

                if pts:
                    xs = [p[0] + ox for p in pts]
                    ys = [p[1] + oy for p in pts]

                    if off_pts:
                        line, = self.ax.plot(xs, ys, color=color, linewidth=1.0,
                                             linestyle='--', alpha=0.45, zorder=2)
                        oxs = [p[0] + ox for p in off_pts]
                        oys = [p[1] + oy for p in off_pts]
                        off_line, = self.ax.plot(oxs, oys, color=color, linewidth=1.8, zorder=3)
                        sx, sy = oxs[0], oys[0]
                    else:
                        line, = self.ax.plot(xs, ys, color=color, linewidth=1.5, zorder=2)
                        off_line = None
                        sx, sy = xs[0], ys[0]

                    marker, = self.ax.plot(sx, sy, 'o', color=color, markersize=8, zorder=4)
                    lbl = self.ax.annotate(
                        f' {path_global_idx + 1}',
                        xy=(sx, sy),
                        fontsize=9, fontweight='bold', color=color, zorder=5,
                        annotation_clip=False,
                    )
                    art.update(line=line, offset_line=off_line, marker=marker, label=lbl)

                self._all_artists.append(art)
                path_global_idx += 1

        self._legend_text = self.ax.text(
            0.01, 0.99, '\n'.join(legend_lines),
            transform=self.ax.transAxes,
            fontsize=8, verticalalignment='top',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='white', alpha=0.85)
        )
        n_files = len(self.dxf_entries)
        self.ax.set_title(
            f'{n_files}ファイル  {path_global_idx}パス  |  '
            f'マシン: {MACHINE_W} × {MACHINE_H} mm'
        )
        self.canvas_widget.draw()

    def _update_torch_display(self, x, y):
        self.pos_x_label.config(text=f'{x:9.3f} mm')
        self.pos_y_label.config(text=f'{y:9.3f} mm')
        self.torch_cross.set_data([x], [y])
        self.torch_dot.set_data([x], [y])
        self.ax.set_title(
            f'トーチ位置: X={x:.3f}  Y={y:.3f} mm  |  '
            f'マシン: {MACHINE_W} × {MACHINE_H} mm'
        )
        self.canvas_widget.draw_idle()

    # ------------------------------------------------------------------ settings tab
    def _build_settings_tab(self, parent):
        def add_entry(p, label, var):
            ttk.Label(p, text=label).pack(anchor=tk.W, padx=6, pady=(4, 0))
            ttk.Entry(p, textvariable=var).pack(fill=tk.X, padx=6, pady=(0, 2))

        sf = ttk.LabelFrame(parent, text='プラズマ設定')
        sf.pack(fill=tk.X, padx=5, pady=(5, 5))

        self.feed_rate     = tk.StringVar(value='3000')
        self.pierce_delay  = tk.StringVar(value='0.5')
        self.kerf_width    = tk.StringVar(value='1.5')
        self.lead_in_length  = tk.StringVar(value='5.0')
        self.lead_out_length = tk.StringVar(value='3.0')

        add_entry(sf, 'カット速度 (mm/min)', self.feed_rate)
        add_entry(sf, 'ピアス遅延 (秒)', self.pierce_delay)
        add_entry(sf, 'カーフ幅 (mm)', self.kerf_width)
        add_entry(sf, 'リードイン長さ (mm)', self.lead_in_length)

        ttk.Label(sf, text='リードイン種類').pack(anchor=tk.W, padx=6, pady=(4, 0))
        self.lead_in_type = tk.StringVar(value='line')
        ttk.Radiobutton(sf, text='直線', variable=self.lead_in_type, value='line').pack(anchor=tk.W, padx=18)
        ttk.Radiobutton(sf, text='円弧', variable=self.lead_in_type, value='arc').pack(anchor=tk.W, padx=18)

        add_entry(sf, 'リードアウト長さ (mm)', self.lead_out_length)

        # ---- File list ----
        ff = ttk.LabelFrame(parent, text='読み込みファイル')
        ff.pack(fill=tk.X, padx=5, pady=(0, 5))

        list_frame = ttk.Frame(ff)
        list_frame.pack(fill=tk.X, padx=5, pady=(4, 0))

        self.file_listbox = tk.Listbox(list_frame, height=4, selectmode=tk.SINGLE,
                                       font=('', 9))
        sb = ttk.Scrollbar(list_frame, orient=tk.VERTICAL,
                           command=self.file_listbox.yview)
        self.file_listbox.config(yscrollcommand=sb.set)
        self.file_listbox.pack(side=tk.LEFT, fill=tk.X, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.file_listbox.bind('<<ListboxSelect>>', self._on_file_select)

        btn_row = ttk.Frame(ff)
        btn_row.pack(fill=tk.X, padx=5, pady=4)
        ttk.Button(btn_row, text='DXFを追加',
                   command=self.open_dxf).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        ttk.Button(btn_row, text='選択を削除',
                   command=self._delete_selected_file).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))

        # ---- Info ----
        info = ttk.LabelFrame(parent, text='ファイル情報')
        info.pack(fill=tk.X, padx=5, pady=(0, 5))
        self.info_label = ttk.Label(info, text='ファイル未選択',
                                    wraplength=260, justify=tk.LEFT)
        self.info_label.pack(padx=5, pady=5)

        ttk.Button(parent, text='Gコードを保存',
                   command=self.save_gcode).pack(fill=tk.X, padx=5, pady=2)

        # ---- DXF offset (per selected file) ----
        of = ttk.LabelFrame(parent, text='選択ファイルのオフセット (mm)')
        of.pack(fill=tk.X, padx=5, pady=(8, 5))

        self.offset_x = tk.StringVar(value='0')
        self.offset_y = tk.StringVar(value='0')

        ox_row = ttk.Frame(of)
        ox_row.pack(fill=tk.X, padx=6, pady=(4, 0))
        ttk.Label(ox_row, text='X :').pack(side=tk.LEFT)
        ttk.Entry(ox_row, textvariable=self.offset_x, width=8).pack(side=tk.LEFT, padx=4)
        ttk.Button(ox_row, text='-10', width=4,
                   command=lambda: self._change_offset('x', -10)).pack(side=tk.LEFT, padx=1)
        ttk.Button(ox_row, text='+10', width=4,
                   command=lambda: self._change_offset('x', 10)).pack(side=tk.LEFT, padx=1)

        oy_row = ttk.Frame(of)
        oy_row.pack(fill=tk.X, padx=6, pady=(2, 4))
        ttk.Label(oy_row, text='Y :').pack(side=tk.LEFT)
        ttk.Entry(oy_row, textvariable=self.offset_y, width=8).pack(side=tk.LEFT, padx=4)
        ttk.Button(oy_row, text='-10', width=4,
                   command=lambda: self._change_offset('y', -10)).pack(side=tk.LEFT, padx=1)
        ttk.Button(oy_row, text='+10', width=4,
                   command=lambda: self._change_offset('y', 10)).pack(side=tk.LEFT, padx=1)

        ttk.Button(of, text='プレビューに反映',
                   command=self._apply_offset).pack(fill=tk.X, padx=5, pady=(0, 4))

        # ---- serial ----
        cf = ttk.LabelFrame(parent, text='シリアル接続 (GRBL)')
        cf.pack(fill=tk.X, padx=5, pady=(8, 5))

        row1 = ttk.Frame(cf)
        row1.pack(fill=tk.X, padx=5, pady=(4, 0))
        ttk.Label(row1, text='COMポート:').pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(row1, textvariable=self.port_var, width=9)
        self.port_cb.pack(side=tk.LEFT, padx=4)
        ttk.Button(row1, text='更新', command=self._refresh_ports, width=4).pack(side=tk.LEFT)

        row2 = ttk.Frame(cf)
        row2.pack(fill=tk.X, padx=5, pady=(2, 4))
        ttk.Label(row2, text='ボーレート:').pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value='115200')
        ttk.Combobox(row2, textvariable=self.baud_var,
                     values=['9600', '38400', '115200'], width=9).pack(side=tk.LEFT, padx=4)

        self.connect_btn = ttk.Button(cf, text='接続', command=self._toggle_connect)
        self.connect_btn.pack(fill=tk.X, padx=5, pady=2)
        self.conn_status = ttk.Label(cf, text='未接続', foreground='gray')
        self.conn_status.pack(padx=5, pady=(0, 4))

        self._refresh_ports()

    # ------------------------------------------------------------------ control tab
    def _build_control_tab(self, parent):
        pf = ttk.LabelFrame(parent, text='現在のトーチ位置')
        pf.pack(fill=tk.X, padx=5, pady=(5, 5))

        grid = ttk.Frame(pf)
        grid.pack(padx=10, pady=6)
        font_label = ('', 13, 'bold')
        font_value = ('Courier', 14, 'bold')

        ttk.Label(grid, text='X :', font=font_label).grid(row=0, column=0, sticky=tk.E, padx=4)
        self.pos_x_label = ttk.Label(grid, text='    0.000 mm',
                                     font=font_value, foreground='#1565C0')
        self.pos_x_label.grid(row=0, column=1, sticky=tk.W)

        ttk.Label(grid, text='Y :', font=font_label).grid(row=1, column=0, sticky=tk.E, padx=4)
        self.pos_y_label = ttk.Label(grid, text='    0.000 mm',
                                     font=font_value, foreground='#1565C0')
        self.pos_y_label.grid(row=1, column=1, sticky=tk.W)

        step_f = ttk.LabelFrame(parent, text='移動量 (mm)')
        step_f.pack(fill=tk.X, padx=5, pady=(0, 5))

        step_row = ttk.Frame(step_f)
        step_row.pack(pady=4, fill=tk.X, padx=6)
        ttk.Label(step_row, text='移動量:').pack(side=tk.LEFT)
        self.jog_step = tk.StringVar(value='10')
        ttk.Entry(step_row, textvariable=self.jog_step, width=7).pack(side=tk.LEFT, padx=4)
        ttk.Button(step_row, text='-10', width=4,
                   command=lambda: self._change_step(-10)).pack(side=tk.LEFT, padx=1)
        ttk.Button(step_row, text='+10', width=4,
                   command=lambda: self._change_step(10)).pack(side=tk.LEFT, padx=1)

        speed_row = ttk.Frame(step_f)
        speed_row.pack(fill=tk.X, padx=6, pady=(0, 4))
        ttk.Label(speed_row, text='速度 (mm/min):').pack(side=tk.LEFT)
        self.jog_speed = tk.StringVar(value='3000')
        ttk.Entry(speed_row, textvariable=self.jog_speed, width=8).pack(side=tk.LEFT, padx=4)

        jf = ttk.LabelFrame(parent, text='ジョグ（手動移動）')
        jf.pack(fill=tk.X, padx=5, pady=(0, 5))

        btn_grid = ttk.Frame(jf)
        btn_grid.pack(pady=8)
        W = 5

        ttk.Button(btn_grid, text='Y+', width=W,
                   command=lambda: self._jog(0, 1)).grid(row=0, column=1, padx=3, pady=3)
        ttk.Button(btn_grid, text='X-', width=W,
                   command=lambda: self._jog(-1, 0)).grid(row=1, column=0, padx=3, pady=3)
        ttk.Button(btn_grid, text='⌂\n原点へ', width=W,
                   command=self._goto_origin).grid(row=1, column=1, padx=3, pady=3)
        ttk.Button(btn_grid, text='X+', width=W,
                   command=lambda: self._jog(1, 0)).grid(row=1, column=2, padx=3, pady=3)
        ttk.Button(btn_grid, text='Y-', width=W,
                   command=lambda: self._jog(0, -1)).grid(row=2, column=1, padx=3, pady=3)

        mf = ttk.LabelFrame(parent, text='マシン制御')
        mf.pack(fill=tk.X, padx=5, pady=(0, 5))

        ttk.Button(mf, text='原点復帰  ($H)',
                   command=self._home).pack(fill=tk.X, padx=5, pady=2)
        ttk.Button(mf, text='ここを原点にする  (G92 X0 Y0)',
                   command=self._set_origin).pack(fill=tk.X, padx=5, pady=2)

        btn_row = ttk.Frame(mf)
        btn_row.pack(fill=tk.X, padx=5, pady=(2, 4))
        ttk.Button(btn_row, text='一時停止  (!)',
                   command=self._feed_hold).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(btn_row, text='再開  (~)',
                   command=self._resume).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        sf2 = ttk.LabelFrame(parent, text='Gコード送信')
        sf2.pack(fill=tk.X, padx=5, pady=(0, 5))

        self.send_btn = ttk.Button(sf2, text='Gコードを機械に送信',
                                   command=self._send_gcode, state=tk.DISABLED)
        self.send_btn.pack(fill=tk.X, padx=5, pady=2)
        self.progress = ttk.Progressbar(sf2, mode='determinate')
        self.progress.pack(fill=tk.X, padx=5, pady=(0, 2))
        self.stop_btn = ttk.Button(sf2, text='緊急停止  (M5 + !)',
                                   command=self._emergency_stop, state=tk.DISABLED)
        self.stop_btn.pack(fill=tk.X, padx=5, pady=(0, 5))

    # ------------------------------------------------------------------ file list helpers
    def _selected_entry_idx(self):
        sel = self.file_listbox.curselection()
        return sel[0] if sel else None

    def _update_file_listbox(self):
        self.file_listbox.delete(0, tk.END)
        for e in self.dxf_entries:
            self.file_listbox.insert(tk.END, f"{e['name']}  ({len(e['paths'])}パス)")

    def _update_info_label(self):
        if not self.dxf_entries:
            self.info_label.config(text='ファイル未選択')
            return
        total = sum(len(e['paths']) for e in self.dxf_entries)
        n_closed = sum(
            sum(1 for p in e['paths'] if p.closed)
            for e in self.dxf_entries
        )
        self.info_label.config(
            text=f'ファイル数: {len(self.dxf_entries)}\n'
                 f'総パス数: {total}\n'
                 f'閉じたパス: {n_closed}'
        )

    def _on_file_select(self, event=None):
        idx = self._selected_entry_idx()
        if idx is not None and idx < len(self.dxf_entries):
            e = self.dxf_entries[idx]
            self.offset_x.set(f"{e['offset_x']:.1f}")
            self.offset_y.set(f"{e['offset_y']:.1f}")

    def _delete_selected_file(self):
        idx = self._selected_entry_idx()
        if idx is None:
            messagebox.showwarning('警告', 'ファイルを選択してください')
            return
        name = self.dxf_entries[idx]['name']
        if not messagebox.askyesno('確認', f'「{name}」を削除しますか？'):
            return
        self.dxf_entries.pop(idx)
        self._update_file_listbox()
        if self.dxf_entries:
            new_idx = min(idx, len(self.dxf_entries) - 1)
            self.file_listbox.selection_set(new_idx)
            self._on_file_select()
        else:
            self.offset_x.set('0')
            self.offset_y.set('0')
        self._draw_paths()
        self._update_info_label()

    # ------------------------------------------------------------------ serial
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning('警告', 'COMポートを選択してください')
            return
        try:
            baud = int(self.baud_var.get())
            self.ser = serial.Serial(port, baud, timeout=1)
            self.ser.flushInput()
            self.connect_btn.config(text='切断')
            self.conn_status.config(text=f'接続中: {port} @ {baud} bps',
                                    foreground='green')
            self.send_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.NORMAL)
            self.polling = True
            threading.Thread(target=self._poll_thread, daemon=True).start()
        except Exception as e:
            messagebox.showerror('接続エラー', str(e))

    def _disconnect(self):
        self.polling = False
        if self.ser:
            self.ser.close()
            self.ser = None
        self.connect_btn.config(text='接続')
        self.conn_status.config(text='未接続', foreground='gray')
        self.send_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        self.progress['value'] = 0

    def _send_serial(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            return
        with self.serial_lock:
            self.ser.write((cmd.strip() + '\n').encode())

    def _send_realtime(self, byte: bytes):
        if not self.ser or not self.ser.is_open:
            return
        with self.serial_lock:
            self.ser.write(byte)

    # ------------------------------------------------------------------ position polling
    def _poll_thread(self):
        while self.polling and self.ser and self.ser.is_open:
            if not self.streaming:
                try:
                    with self.serial_lock:
                        self.ser.write(b'?')
                        resp = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if 'Pos:' in resp:
                        self._parse_position(resp)
                except Exception:
                    pass
            time.sleep(0.5)

    def _parse_position(self, resp):
        m = re.search(r'[MW]Pos:([-\d.]+),([-\d.]+)', resp)
        if m:
            x, y = float(m.group(1)), float(m.group(2))
            self.torch_x, self.torch_y = x, y
            self.root.after(0, lambda: self._update_torch_display(x, y))

    # ------------------------------------------------------------------ helpers
    def _change_step(self, delta):
        try:
            val = float(self.jog_step.get()) + delta
            val = max(0.1, val)
            self.jog_step.set(f'{val:.0f}' if val >= 1 else f'{val:.1f}')
        except ValueError:
            self.jog_step.set('10')

    def _change_offset(self, axis, delta):
        idx = self._selected_entry_idx()
        if idx is None or not self.dxf_entries:
            return
        e = self.dxf_entries[idx]
        key = 'offset_x' if axis == 'x' else 'offset_y'
        e[key] = round(e[key] + delta, 1)
        var = self.offset_x if axis == 'x' else self.offset_y
        var.set(f"{e[key]:.1f}")
        self._draw_paths()

    def _apply_offset(self):
        idx = self._selected_entry_idx()
        if idx is None or not self.dxf_entries:
            return
        try:
            self.dxf_entries[idx]['offset_x'] = float(self.offset_x.get())
            self.dxf_entries[idx]['offset_y'] = float(self.offset_y.get())
        except ValueError:
            pass
        self._draw_paths()

    # ------------------------------------------------------------------ jog / machine
    def _jog(self, dx, dy):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning('警告', '先に接続してください')
            return
        step = float(self.jog_step.get())
        speed = self.jog_speed.get()
        parts = ['$J=G21G91']
        if dx:
            parts.append(f'X{dx * step:.3f}')
        if dy:
            parts.append(f'Y{dy * step:.3f}')
        parts.append(f'F{speed}')
        self._send_serial(''.join(parts))

    def _goto_origin(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning('警告', '先に接続してください')
            return
        self._send_serial('G90 G0 X0 Y0')

    def _home(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning('警告', '先に接続してください')
            return
        if messagebox.askyesno('確認', '原点復帰を実行します ($H)\nホーミングスイッチが必要です。よろしいですか？'):
            self._send_serial('$H')

    def _set_origin(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning('警告', '先に接続してください')
            return
        self._send_serial('G92 X0 Y0')
        self.torch_x = 0.0
        self.torch_y = 0.0
        self.root.after(0, lambda: self._update_torch_display(0.0, 0.0))
        messagebox.showinfo('完了', '現在位置を作業原点 (0, 0) に設定しました')

    def _feed_hold(self):
        self._send_realtime(b'!')

    def _resume(self):
        self._send_realtime(b'~')

    def _emergency_stop(self):
        self.streaming = False
        self._send_realtime(b'!')
        self._send_serial('M5')
        self.send_btn.config(state=tk.NORMAL)
        self.progress['value'] = 0

    # ------------------------------------------------------------------ G-code streaming
    def _send_gcode(self):
        if not self.dxf_entries:
            messagebox.showwarning('警告', 'DXFファイルを先に開いてください')
            return
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning('警告', '先に接続してください')
            return
        if self.streaming:
            return
        try:
            settings = self._get_settings()
        except ValueError:
            messagebox.showerror('エラー', '設定値に無効な数値があります')
            return
        if not messagebox.askyesno('確認', 'Gコードを機械に送信します。\nよろしいですか？'):
            return

        gcode = generate_gcode(self.dxf_entries, settings)
        lines = [l.strip() for l in gcode.split('\n')
                 if l.strip() and not l.strip().startswith(';')]
        self.streaming = True
        self.send_btn.config(state=tk.DISABLED)
        threading.Thread(target=self._stream_thread, args=(lines,), daemon=True).start()

    def _stream_thread(self, lines):
        total = len(lines)
        try:
            for i, line in enumerate(lines):
                if not self.streaming:
                    break
                with self.serial_lock:
                    self.ser.write((line + '\n').encode())
                    resp = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if 'error' in resp.lower():
                    self.root.after(0, lambda r=resp, l=line:
                                    messagebox.showerror('GRBLエラー', f'コマンド: {l}\n応答: {r}'))
                    break
                pct = int((i + 1) / total * 100)
                self.root.after(0, lambda v=pct: self.progress.configure(value=v))
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror('送信エラー', str(e)))
        finally:
            self.streaming = False
            self.root.after(0, lambda: self.send_btn.config(state=tk.NORMAL))

    # ------------------------------------------------------------------ DXF
    def open_dxf(self):
        filename = filedialog.askopenfilename(
            title='DXFファイルを選択',
            filetypes=[('DXF files', '*.dxf'), ('All files', '*.*')]
        )
        if not filename:
            return
        try:
            paths = read_dxf(filename)
            name = os.path.basename(filename)
            self.dxf_entries.append({
                'name': name,
                'paths': paths,
                'offset_x': 0.0,
                'offset_y': 0.0,
            })
            self._update_file_listbox()
            self.file_listbox.selection_clear(0, tk.END)
            self.file_listbox.selection_set(len(self.dxf_entries) - 1)
            self._on_file_select()
            self._draw_paths()
            self._update_info_label()
        except Exception as e:
            messagebox.showerror('エラー', f'DXFの読み込みに失敗しました:\n{e}')

    def _get_settings(self):
        return {
            'feed_rate':      float(self.feed_rate.get()),
            'pierce_delay':   float(self.pierce_delay.get()),
            'kerf_width':     float(self.kerf_width.get()),
            'lead_in_length': float(self.lead_in_length.get()),
            'lead_in_type':   self.lead_in_type.get(),
            'lead_out_length': float(self.lead_out_length.get()),
        }

    def save_gcode(self):
        if not self.dxf_entries:
            messagebox.showwarning('警告', 'DXFファイルを先に開いてください')
            return
        try:
            settings = self._get_settings()
        except ValueError:
            messagebox.showerror('エラー', '設定値に無効な数値があります')
            return
        filename = filedialog.asksaveasfilename(
            title='Gコードを保存',
            defaultextension='.gcode',
            filetypes=[('G-code', '*.gcode *.nc *.tap'), ('All files', '*.*')]
        )
        if not filename:
            return
        try:
            gcode = generate_gcode(self.dxf_entries, settings)
            with open(filename, 'w') as f:
                f.write(gcode)
            messagebox.showinfo('完了', f'Gコードを保存しました:\n{filename}')
        except Exception as e:
            messagebox.showerror('エラー', f'保存に失敗しました:\n{e}')

    def _on_close(self):
        self.streaming = False
        self.polling = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()


if __name__ == '__main__':
    root = tk.Tk()
    app = PlasmaCamApp(root)
    root.mainloop()
