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
import math
import serial
import serial.tools.list_ports

matplotlib.rcParams['font.family'] = ['Yu Gothic', 'MS Gothic', 'Meiryo', 'sans-serif']

from dxf_reader import read_dxf
from gcode_generator import generate_gcode, generate_from_plan
from kerf_offset import compute_offset_points, compute_path_types
import settings_manager

MACHINE_W = 1200
MACHINE_H = 800
COLORS = ['#2196F3', '#4CAF50', '#FF5722', '#9C27B0', '#FF9800']


class PlasmaCamApp:
    def __init__(self, root):
        self.root = root
        self.root.title('Plasma CAM - GRBL')
        self.root.geometry('1500x900')
        self._layout_file = os.path.join(
            os.path.dirname(__file__), 'layouts.json')
        self._layouts = self._load_layout_file()

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

        # 設定ファイルからロード
        self._saved_settings, self._presets = settings_manager.load()

        self._build_menu()
        self._build_ui()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

    # ------------------------------------------------------------------ scroll tab helper
    def _make_scroll_tab(self, notebook, label):
        """スクロール可能なタブを作成し、内側フレームを返す"""
        outer = ttk.Frame(notebook)
        notebook.add(outer, text=label)

        canvas = tk.Canvas(outer, borderwidth=0, highlightthickness=0)
        scrollbar = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        inner = ttk.Frame(canvas)
        win_id = canvas.create_window((0, 0), window=inner, anchor='nw')

        def on_frame_configure(e):
            canvas.configure(scrollregion=canvas.bbox('all'))

        def on_canvas_configure(e):
            canvas.itemconfig(win_id, width=e.width)

        def on_mousewheel(e):
            canvas.yview_scroll(int(-1 * (e.delta / 120)), 'units')

        inner.bind('<Configure>', on_frame_configure)
        canvas.bind('<Configure>', on_canvas_configure)
        canvas.bind('<Enter>', lambda e: canvas.bind_all('<MouseWheel>', on_mousewheel))
        canvas.bind('<Leave>', lambda e: canvas.unbind_all('<MouseWheel>'))

        return inner

    # ------------------------------------------------------------------ menu
    def _build_menu(self):
        mb = tk.Menu(self.root)
        fm = tk.Menu(mb, tearoff=0)
        fm.add_command(label='DXFを追加', command=self.open_dxf)
        fm.add_command(label='Gコードを保存', command=self.save_gcode)
        fm.add_separator()
        fm.add_command(label='終了', command=self._on_close)
        mb.add_cascade(label='ファイル', menu=fm)

        # レイアウトメニュー
        lm = tk.Menu(mb, tearoff=0)
        lm.add_command(label='現在のレイアウトを保存...', command=self._save_layout_as)
        lm.add_command(label='デフォルトに戻す',          command=self._apply_default_layout)
        lm.add_separator()
        self._layout_menu = lm
        self._rebuild_layout_menu()
        mb.add_cascade(label='レイアウト', menu=lm)

        self.root.config(menu=mb)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        """
        レイアウト:
        [ キャンバス ] [ 設定 ] [ 機械制御 ] [ コマンド入力 ]
        [   (大)    ] [      ] [          ] [ コンソール表示]
        """
        h_pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL,
                                sashwidth=5, sashrelief='raised', bg='#aaa')
        h_pane.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # ── 列1: キャンバス ──────────────────────────────
        cf = ttk.LabelFrame(h_pane, text='マシンビュー')
        h_pane.add(cf, minsize=380, stretch='always')

        # キャンバス上部ツールバー
        canvas_tb = tk.Frame(cf, bg='#f0f0f0')
        canvas_tb.pack(fill=tk.X)
        self._goto_mode = tk.BooleanVar(value=False)
        self._goto_btn = tk.Checkbutton(
            canvas_tb, text='🎯 クリックで移動',
            variable=self._goto_mode,
            indicatoron=False,
            selectcolor='#ffcc00',
            bg='#e0e0e0', activebackground='#ffcc00',
            font=('Yu Gothic UI', 9, 'bold'),
            relief='raised', padx=8, pady=3,
            cursor='hand2',
            command=self._on_goto_mode_toggle)
        self._goto_btn.pack(side=tk.LEFT, padx=6, pady=3)
        self._goto_pos_label = tk.Label(
            canvas_tb, text='', bg='#f0f0f0',
            fg='#333', font=('Consolas', 9))
        self._goto_pos_label.pack(side=tk.LEFT, padx=8)

        self.fig, self.ax = plt.subplots(figsize=(8, 7))
        self.fig.subplots_adjust(bottom=0.04, top=0.96, left=0.08, right=0.98)
        self.canvas_widget = FigureCanvasTkAgg(self.fig, master=cf)
        self.canvas_widget.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        ttk.Label(cf, text='左ドラッグ:DXF移動  右ドラッグ:視点  ホイール:ズーム  🎯ONでクリック移動',
                  foreground='#666', font=('', 8)).pack(pady=1)
        self._init_canvas()
        self._connect_canvas_events()

        # ── 列2: 設定（スクロール対応） ───────────────────────
        col2 = ttk.LabelFrame(h_pane, text='⚙ 設定')
        h_pane.add(col2, minsize=200, stretch='never')

        _cv2  = tk.Canvas(col2, borderwidth=0, highlightthickness=0)
        _sb2  = ttk.Scrollbar(col2, orient=tk.VERTICAL, command=_cv2.yview)
        _cv2.configure(yscrollcommand=_sb2.set)
        _sb2.pack(side=tk.RIGHT, fill=tk.Y)
        _cv2.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        t1 = ttk.Frame(_cv2)
        _wid2 = _cv2.create_window((0, 0), window=t1, anchor='nw')

        def _cfg2(e): _cv2.configure(scrollregion=_cv2.bbox('all'))
        def _cw2(e):  _cv2.itemconfig(_wid2, width=e.width)
        def _mw2(e):  _cv2.yview_scroll(int(-1*(e.delta/120)), 'units')
        t1.bind('<Configure>', _cfg2)
        _cv2.bind('<Configure>', _cw2)
        _cv2.bind('<Enter>', lambda e: _cv2.bind_all('<MouseWheel>', _mw2))
        _cv2.bind('<Leave>', lambda e: _cv2.unbind_all('<MouseWheel>'))

        # ── 列3: 機械制御 ─────────────────────────────────
        col3 = ttk.LabelFrame(h_pane, text='🎮 機械制御')
        h_pane.add(col3, minsize=200, stretch='never')
        t2 = col3   # _build_control_tab に渡す

        # ── 列4: コマンド入力 + コンソール (縦2分割) ─────
        col4 = tk.PanedWindow(h_pane, orient=tk.VERTICAL,
                              sashwidth=5, sashrelief='raised', bg='#aaa')
        h_pane.add(col4, minsize=220, stretch='never')

        # 上: コマンド入力
        cmd_frm = ttk.LabelFrame(col4, text='⌨ GRBLコマンド入力')
        col4.add(cmd_frm, minsize=80, stretch='never')

        self.manual_cmd = tk.StringVar()
        self._cmd_hist = []
        self._cmd_hist_idx = 0

        in_row = ttk.Frame(cmd_frm)
        in_row.pack(fill=tk.X, padx=4, pady=(4, 2))
        ttk.Label(in_row, text='>>>',
                  font=('Consolas', 10, 'bold'),
                  foreground='#2255cc').pack(side=tk.LEFT, padx=(0, 3))
        cmd_entry = ttk.Entry(in_row, textvariable=self.manual_cmd,
                              font=('Consolas', 10))
        cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        cmd_entry.bind('<Return>', lambda e: self._send_manual_cmd())
        cmd_entry.bind('<Up>',     lambda e: self._cmd_history(-1))
        cmd_entry.bind('<Down>',   lambda e: self._cmd_history(1))

        btn_row = ttk.Frame(cmd_frm)
        btn_row.pack(fill=tk.X, padx=4, pady=(0, 4))
        ttk.Button(btn_row, text='送信',
                   command=self._send_manual_cmd).pack(side=tk.LEFT, padx=(0, 6))
        for lbl, cmd in [('$X', '$X'), ('?', '?'), ('$$', '$$'), ('$H', '$H')]:
            ttk.Button(btn_row, text=lbl, width=4,
                       command=lambda c=cmd: self._quick_cmd(c)
                       ).pack(side=tk.LEFT, padx=1)

        # 下: コンソール表示
        con_frm = tk.Frame(col4, bg='#0a0a0a')
        col4.add(con_frm, minsize=120, stretch='always')

        con_top = tk.Frame(con_frm, bg='#1a1a2e', height=22)
        con_top.pack(fill=tk.X)
        con_top.pack_propagate(False)
        tk.Label(con_top, text='📟 コンソール表示',
                 bg='#1a1a2e', fg='#88ccff',
                 font=('Yu Gothic UI', 9, 'bold')).pack(side=tk.LEFT, padx=6)
        tk.Button(con_top, text='クリア', bg='#2a2a4e', fg='#ccc',
                  relief='flat', font=('', 8), cursor='hand2',
                  command=self._clear_console).pack(side=tk.RIGHT, padx=4)

        log_f = tk.Frame(con_frm, bg='#0a0a0a')
        log_f.pack(fill=tk.BOTH, expand=True)
        con_sb = tk.Scrollbar(log_f)
        self.console = tk.Text(log_f, bg='#0a0a0a', fg='#00ff88',
                               font=('Consolas', 9), wrap='word',
                               insertbackground='white', state=tk.DISABLED,
                               yscrollcommand=con_sb.set, bd=0)
        con_sb.config(command=self.console.yview)
        con_sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.console.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        # PanedWindow への参照を保持（レイアウト保存用）
        self._h_pane = h_pane
        self._col4   = col4

        # 起動時にレイアウトを適用
        def _init_sash(e=None):
            w = self.root.winfo_width()
            h = self.root.winfo_height()
            if w < 600:
                return
            saved = self._layouts.get('__last__')
            if saved:
                self._apply_layout_dict(saved)
            else:
                self._apply_default_layout()
            self.root.unbind('<Map>')
        self.root.bind('<Map>', _init_sash)

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

    def _on_goto_mode_toggle(self):
        if self._goto_mode.get():
            self._goto_btn.config(relief='sunken', bg='#ffcc00')
            self._goto_pos_label.config(text='キャンバスをクリックして移動先を指定')
        else:
            self._goto_btn.config(relief='raised', bg='#e0e0e0')
            self._goto_pos_label.config(text='')

    def _on_press(self, event):
        if event.inaxes != self.ax:
            return

        # 🎯 移動モードON かつ 左クリック → その座標へ移動
        if self._goto_mode.get() and event.button == 1:
            x = round(event.xdata, 3)
            y = round(event.ydata, 3)
            self._goto_pos_label.config(
                text=f'→ X={x:.3f}  Y={y:.3f}')
            if self.ser and self.ser.is_open:
                self._send_serial(f'G0 X{x:.3f} Y{y:.3f}')
                self._log(f'>>> 🎯 G0 X{x:.3f} Y{y:.3f}', 'send')
            else:
                self._goto_pos_label.config(
                    text=f'⚠ 未接続  X={x:.3f}  Y={y:.3f}')
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

    def highlight_path(self, entry, path_idx):
        """指定パスをハイライト表示してキャンバスを中央寄せ"""
        if entry is None or path_idx is None:
            return
        ox = entry.get('offset_x', 0.0)
        oy = entry.get('offset_y', 0.0)
        path = entry['paths'][path_idx]
        pts  = path.get_display_points()
        if not pts:
            return

        xs = [p[0] + ox for p in pts]
        ys = [p[1] + oy for p in pts]

        # 既存ハイライトを消す
        if hasattr(self, '_highlight_artist') and self._highlight_artist:
            try:
                self._highlight_artist.remove()
            except Exception:
                pass
            self._highlight_artist = None
        if hasattr(self, '_highlight_bbox') and self._highlight_bbox:
            try:
                self._highlight_bbox.remove()
            except Exception:
                pass
            self._highlight_bbox = None

        # ハイライト線（黄色・太め）
        hl, = self.ax.plot(xs, ys,
                           color='#ffff00', linewidth=3.5,
                           alpha=0.9, zorder=10, linestyle='-')
        self._highlight_artist = hl

        # 開始点マーカー
        bx, = self.ax.plot([xs[0]], [ys[0]], 'o',
                           color='#ff4444', markersize=10, zorder=11)
        self._highlight_bbox = bx

        # キャンバスをそのパスに中央寄せ（ズーム付き）
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        padx = max((xmax - xmin) * 0.6, 30)
        pady = max((ymax - ymin) * 0.6, 30)
        self.ax.set_xlim(xmin - padx, xmax + padx)
        self.ax.set_ylim(ymin - pady, ymax + pady)

        self.canvas_widget.draw_idle()

    def clear_highlight(self):
        for attr in ('_highlight_artist', '_highlight_bbox'):
            artist = getattr(self, attr, None)
            if artist:
                try:
                    artist.remove()
                except Exception:
                    pass
                setattr(self, attr, None)
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

            # 内側(穴)→外側 の順に並べ替え（Gコード生成と順番を合わせる）
            order = sorted(range(len(paths)),
                           key=lambda idx: (0 if inner_flags[idx] else 1))
            paths_sorted = [paths[idx]       for idx in order]
            flags_sorted = [inner_flags[idx] for idx in order]

            for j, path in enumerate(paths_sorted):
                pts = path.get_display_points()
                is_inner = flags_sorted[j]
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

    def _fit_view(self):
        """全パスが収まるようにビューをフィット"""
        if not self.dxf_entries:
            self._reset_view()
            return
        all_xs, all_ys = [], []
        for entry in self.dxf_entries:
            ox, oy = entry['offset_x'], entry['offset_y']
            for path in entry['paths']:
                for p in path.get_display_points():
                    all_xs.append(p[0] + ox)
                    all_ys.append(p[1] + oy)
        if not all_xs:
            self._reset_view()
            return
        xmin, xmax = min(all_xs), max(all_xs)
        ymin, ymax = min(all_ys), max(all_ys)
        mx = max((xmax - xmin) * 0.15, 30)
        my = max((ymax - ymin) * 0.15, 30)
        self.ax.set_xlim(xmin - mx, xmax + mx)
        self.ax.set_ylim(ymin - my, ymax + my)
        self.canvas_widget.draw_idle()

    def _reset_view(self):
        """マシン全体が見えるデフォルトビューに戻す"""
        self.ax.set_xlim(-80, MACHINE_W + 80)
        self.ax.set_ylim(-60, MACHINE_H + 60)
        self.canvas_widget.draw_idle()

    def _update_torch_display(self, x, y, z=None):
        self.pos_x_label.config(text=f'{x:9.3f} mm')
        self.pos_y_label.config(text=f'{y:9.3f} mm')
        if z is not None and hasattr(self, 'pos_z_label'):
            self.pos_z_label.config(text=f'{z:9.3f} mm')
        self.torch_cross.set_data([x], [y])
        self.torch_dot.set_data([x], [y])
        z_str = f'  Z={z:.3f}' if z is not None else ''
        self.ax.set_title(
            f'トーチ位置: X={x:.3f}  Y={y:.3f}{z_str} mm  |  '
            f'マシン: {MACHINE_W} × {MACHINE_H} mm'
        )
        self.canvas_widget.draw_idle()

    # ------------------------------------------------------------------ settings tab
    def _build_settings_tab(self, parent):
        def g(p, label, var, r, c):
            ttk.Label(p, text=label, font=('', 8)).grid(row=r, column=c*2,   sticky='e', padx=(4,2), pady=2)
            ttk.Entry(p, textvariable=var, width=7).grid(row=r, column=c*2+1, sticky='w', padx=(0,6), pady=2)

        # ---- プラズマ設定 (2列グリッド) ----
        sf = ttk.LabelFrame(parent, text='プラズマ設定')
        sf.pack(fill=tk.X, padx=5, pady=(4, 2))

        s = self._saved_settings
        self.feed_rate        = tk.StringVar(value=s.get('feed_rate',       '3000'))
        self.pierce_delay     = tk.StringVar(value='0.5')   # 後方互換のため保持
        self.kerf_width       = tk.StringVar(value=s.get('kerf_width',      '1.5'))
        self.lead_in_length   = tk.StringVar(value=s.get('lead_in_length',  '5.0'))
        self.lead_out_length  = tk.StringVar(value=s.get('lead_out_length', '3.0'))

        g(sf, 'カット速度(mm/min)', self.feed_rate,       0, 0)
        g(sf, 'カーフ幅(mm)',       self.kerf_width,      0, 1)
        g(sf, 'リードイン(mm)',     self.lead_in_length,  1, 0)
        g(sf, 'リードアウト(mm)',   self.lead_out_length, 1, 1)

        li_f = ttk.Frame(sf)
        li_f.grid(row=2, column=0, columnspan=4, sticky='w', padx=6, pady=2)
        self.lead_in_type = tk.StringVar(value=s.get('lead_in_type', 'line'))
        ttk.Label(li_f, text='リードイン種類:', font=('',8)).pack(side=tk.LEFT)
        ttk.Radiobutton(li_f, text='直線', variable=self.lead_in_type, value='line').pack(side=tk.LEFT)
        ttk.Radiobutton(li_f, text='円弧', variable=self.lead_in_type, value='arc').pack(side=tk.LEFT)

        # ---- スマートピアシング ----
        pf = ttk.LabelFrame(parent, text='スマートピアシング')
        pf.pack(fill=tk.X, padx=5, pady=(2, 2))

        self.hot_start_distance = tk.StringVar(value=s.get('hot_start_distance', '50.0'))
        self.hot_start_time     = tk.StringVar(value=s.get('hot_start_time',      '2.5'))
        self.hot_pierce_ms      = tk.StringVar(value=s.get('hot_pierce_ms',       '800'))
        self.cold_pierce_ms     = tk.StringVar(value=s.get('cold_pierce_ms',     '2400'))
        self.rapid_speed        = tk.StringVar(value=s.get('rapid_speed',         '5000'))

        # プリセット行
        pr_row = ttk.Frame(pf)
        pr_row.pack(fill=tk.X, padx=4, pady=(4, 2))
        ttk.Label(pr_row, text='プリセット:', font=('',8)).pack(side=tk.LEFT)
        self._preset_var = tk.StringVar()
        self._preset_cb  = ttk.Combobox(pr_row, textvariable=self._preset_var,
                                         width=7, state='readonly')
        self._preset_cb.pack(side=tk.LEFT, padx=3)
        self._preset_cb.bind('<<ComboboxSelected>>', self._apply_preset)
        ttk.Button(pr_row, text='保存', width=4,
                   command=self._save_preset).pack(side=tk.LEFT, padx=1)
        ttk.Button(pr_row, text='削除', width=4,
                   command=self._delete_preset).pack(side=tk.LEFT, padx=1)
        self._refresh_preset_list()

        # 判定条件グリッド
        cond = ttk.Frame(pf)
        cond.pack(fill=tk.X, padx=4, pady=1)
        g(cond, 'ホット判定距離(mm)', self.hot_start_distance, 0, 0)
        g(cond, 'ホット判定時間(s)',  self.hot_start_time,     0, 1)
        g(cond, 'ラピッド速度(mm/min)', self.rapid_speed,      1, 0)

        # ピアシング時間グリッド
        pt = ttk.Frame(pf)
        pt.pack(fill=tk.X, padx=4, pady=(1,4))
        g(pt, 'ホット時(ms)',   self.hot_pierce_ms,  0, 0)
        g(pt, 'コールド時(ms)', self.cold_pierce_ms, 0, 1)

        # ---- ファイルリスト ----
        ff = ttk.LabelFrame(parent, text='DXFファイル')
        ff.pack(fill=tk.X, padx=5, pady=(2, 2))

        list_frame = ttk.Frame(ff)
        list_frame.pack(fill=tk.X, padx=5, pady=(3, 0))
        self.file_listbox = tk.Listbox(list_frame, height=3, selectmode=tk.SINGLE, font=('', 9))
        sb = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.file_listbox.yview)
        self.file_listbox.config(yscrollcommand=sb.set)
        self.file_listbox.pack(side=tk.LEFT, fill=tk.X, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.file_listbox.bind('<<ListboxSelect>>', self._on_file_select)

        btn_row = ttk.Frame(ff)
        btn_row.pack(fill=tk.X, padx=5, pady=3)
        ttk.Button(btn_row, text='DXF追加', command=self.open_dxf).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0,2))
        ttk.Button(btn_row, text='削除',    command=self._delete_selected_file).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2,0))

        self.info_label = ttk.Label(ff, text='未選択', font=('', 8))
        self.info_label.pack(padx=5, pady=(0,3))

        # ---- パス一覧 + 移動ボタン ----
        pf = ttk.LabelFrame(parent, text='パス一覧')
        pf.pack(fill=tk.X, padx=5, pady=(0, 2))

        path_list_frame = ttk.Frame(pf)
        path_list_frame.pack(fill=tk.X, padx=4, pady=(3,0))
        self.path_listbox = tk.Listbox(path_list_frame, height=4,
                                       selectmode=tk.SINGLE, font=('Consolas', 8))
        psb = ttk.Scrollbar(path_list_frame, orient=tk.VERTICAL,
                            command=self.path_listbox.yview)
        self.path_listbox.config(yscrollcommand=psb.set)
        self.path_listbox.pack(side=tk.LEFT, fill=tk.X, expand=True)
        psb.pack(side=tk.RIGHT, fill=tk.Y)

        ttk.Button(pf, text='🎯  ここへ移動',
                   command=self._goto_selected_path).pack(
                       fill=tk.X, padx=5, pady=(3, 5))

        btn_row_gc = ttk.Frame(parent)
        btn_row_gc.pack(fill=tk.X, padx=5, pady=2)
        ttk.Button(btn_row_gc, text='⚙ CAM編集',
                   command=self.show_cam_editor).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0,2))
        ttk.Button(btn_row_gc, text='▶ プレビュー',
                   command=self.show_motion_preview).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2,2))
        ttk.Button(btn_row_gc, text='🔍 Gコード',
                   command=self.show_gcode_preview).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2,2))
        ttk.Button(btn_row_gc, text='💾 保存',
                   command=self.save_gcode).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2,0))

        btn_row_view = ttk.Frame(parent)
        btn_row_view.pack(fill=tk.X, padx=5, pady=(0, 2))
        ttk.Button(btn_row_view, text='🔭 全体を表示',
                   command=self._fit_view).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0,2))
        ttk.Button(btn_row_view, text='⌂ マシン全体',
                   command=self._reset_view).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2,0))

        # ---- オフセット ----
        of = ttk.LabelFrame(parent, text='オフセット (mm)')
        of.pack(fill=tk.X, padx=5, pady=(2, 2))

        self.offset_x = tk.StringVar(value='0')
        self.offset_y = tk.StringVar(value='0')

        ox_row = ttk.Frame(of)
        ox_row.pack(fill=tk.X, padx=4, pady=2)
        ttk.Label(ox_row, text='X:').pack(side=tk.LEFT)
        ttk.Entry(ox_row, textvariable=self.offset_x, width=7).pack(side=tk.LEFT, padx=3)
        ttk.Button(ox_row, text='-10', width=4, command=lambda: self._change_offset('x',-10)).pack(side=tk.LEFT, padx=1)
        ttk.Button(ox_row, text='+10', width=4, command=lambda: self._change_offset('x', 10)).pack(side=tk.LEFT, padx=1)

        oy_row = ttk.Frame(of)
        oy_row.pack(fill=tk.X, padx=4, pady=2)
        ttk.Label(oy_row, text='Y:').pack(side=tk.LEFT)
        ttk.Entry(oy_row, textvariable=self.offset_y, width=7).pack(side=tk.LEFT, padx=3)
        ttk.Button(oy_row, text='-10', width=4, command=lambda: self._change_offset('y',-10)).pack(side=tk.LEFT, padx=1)
        ttk.Button(oy_row, text='+10', width=4, command=lambda: self._change_offset('y', 10)).pack(side=tk.LEFT, padx=1)
        ttk.Button(of, text='反映', command=self._apply_offset).pack(fill=tk.X, padx=5, pady=(0,3))

        # ---- シリアル接続 ----
        cf = ttk.LabelFrame(parent, text='シリアル接続 (GRBL)')
        cf.pack(fill=tk.X, padx=5, pady=(2, 4))

        row1 = ttk.Frame(cf)
        row1.pack(fill=tk.X, padx=5, pady=(3,0))
        ttk.Label(row1, text='COM:').pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(row1, textvariable=self.port_var, width=8)
        self.port_cb.pack(side=tk.LEFT, padx=3)
        ttk.Label(row1, text='Baud:').pack(side=tk.LEFT, padx=(4,0))
        self.baud_var = tk.StringVar(value='115200')
        ttk.Combobox(row1, textvariable=self.baud_var, values=['9600','38400','115200'], width=8).pack(side=tk.LEFT, padx=3)
        ttk.Button(row1, text='更新', command=self._refresh_ports, width=4).pack(side=tk.LEFT)

        row2 = ttk.Frame(cf)
        row2.pack(fill=tk.X, padx=5, pady=(2,3))
        self.connect_btn = ttk.Button(row2, text='接続', command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0,2))
        self.offline_btn = ttk.Button(row2, text='オフライン', command=self._toggle_offline)
        self.offline_btn.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2,0))

        self.conn_status = ttk.Label(cf, text='未接続', foreground='gray', font=('',8))
        self.conn_status.pack(padx=5, pady=(0,3))

        self._refresh_ports()

    # ------------------------------------------------------------------ preset helpers
    def _refresh_preset_list(self):
        names = list(self._presets.keys())
        self._preset_cb['values'] = names
        if names and not self._preset_var.get():
            self._preset_var.set(names[0])

    def _apply_preset(self, event=None):
        name = self._preset_var.get()
        if name not in self._presets:
            return
        p = self._presets[name]
        self.hot_start_distance.set(p.get('hot_start_distance', '50.0'))
        self.hot_start_time.set(p.get('hot_start_time',     '2.5'))
        self.hot_pierce_ms.set(p.get('hot_pierce_ms',       '800'))
        self.cold_pierce_ms.set(p.get('cold_pierce_ms',    '2400'))

    def _save_preset(self):
        from tkinter.simpledialog import askstring
        name = askstring('プリセット保存', 'プリセット名を入力してください:',
                         parent=self.root)
        if not name or not name.strip():
            return
        name = name.strip()
        self._presets[name] = {
            'hot_start_distance': self.hot_start_distance.get(),
            'hot_start_time':     self.hot_start_time.get(),
            'hot_pierce_ms':      self.hot_pierce_ms.get(),
            'cold_pierce_ms':     self.cold_pierce_ms.get(),
        }
        try:
            raw = {k: str(v) for k, v in self._get_settings().items()}
            settings_manager.save(raw, self._presets)
        except Exception:
            pass
        self._refresh_preset_list()
        self._preset_var.set(name)
        messagebox.showinfo('保存完了', f'プリセット「{name}」を保存しました。')

    def _delete_preset(self):
        name = self._preset_var.get()
        if not name:
            messagebox.showwarning('警告', 'プリセットを選択してください')
            return
        if name not in self._presets:
            messagebox.showwarning('警告', f'プリセット「{name}」が見つかりません')
            return
        if not messagebox.askyesno('確認', f'プリセット「{name}」を削除しますか？'):
            return
        del self._presets[name]
        try:
            raw = {k: str(v) for k, v in self._get_settings().items()}
            settings_manager.save(raw, self._presets)
        except Exception:
            pass
        self._refresh_preset_list()
        self._preset_var.set(self._preset_cb['values'][0] if self._preset_cb['values'] else '')
        messagebox.showinfo('削除完了', f'プリセット「{name}」を削除しました。')

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

        ttk.Label(grid, text='Z :', font=font_label).grid(row=2, column=0, sticky=tk.E, padx=4)
        self.pos_z_label = ttk.Label(grid, text='    0.000 mm',
                                     font=font_value, foreground='#00897B')
        self.pos_z_label.grid(row=2, column=1, sticky=tk.W)

        self.grbl_state_label = ttk.Label(grid, text='状態: --',
                                          font=('', 10), foreground='gray')
        self.grbl_state_label.grid(row=3, column=0, columnspan=2, pady=(6, 0))

        step_f = ttk.LabelFrame(parent, text='移動量 (mm)')
        step_f.pack(fill=tk.X, padx=5, pady=(0, 5))

        step_row = ttk.Frame(step_f)
        step_row.pack(pady=4, fill=tk.X, padx=6)
        ttk.Label(step_row, text='XY:').pack(side=tk.LEFT)
        self.jog_step = tk.StringVar(value='10')
        ttk.Entry(step_row, textvariable=self.jog_step, width=6).pack(side=tk.LEFT, padx=3)
        ttk.Button(step_row, text='-10', width=4,
                   command=lambda: self._change_step(-10)).pack(side=tk.LEFT, padx=1)
        ttk.Button(step_row, text='+10', width=4,
                   command=lambda: self._change_step(10)).pack(side=tk.LEFT, padx=1)

        # Z軸専用移動距離
        z_step_row = ttk.Frame(step_f)
        z_step_row.pack(fill=tk.X, padx=6, pady=(0, 2))
        ttk.Label(z_step_row, text='Z :', foreground='#00897B',
                  font=('', 9, 'bold')).pack(side=tk.LEFT)
        self.jog_step_z = tk.StringVar(value='1')
        ttk.Entry(z_step_row, textvariable=self.jog_step_z, width=6).pack(side=tk.LEFT, padx=3)
        ttk.Button(z_step_row, text='-1',  width=4,
                   command=lambda: self._change_step_z(-1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(z_step_row, text='+1',  width=4,
                   command=lambda: self._change_step_z(1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(z_step_row, text='+10', width=4,
                   command=lambda: self._change_step_z(10)).pack(side=tk.LEFT, padx=1)

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

        # XY ジョグ
        ttk.Button(btn_grid, text='Y+', width=W,
                   command=lambda: self._jog(0, 1, 0)).grid(row=0, column=1, padx=3, pady=3)
        ttk.Button(btn_grid, text='X-', width=W,
                   command=lambda: self._jog(-1, 0, 0)).grid(row=1, column=0, padx=3, pady=3)
        ttk.Button(btn_grid, text='⌂\n原点へ', width=W,
                   command=self._goto_origin).grid(row=1, column=1, padx=3, pady=3)
        ttk.Button(btn_grid, text='X+', width=W,
                   command=lambda: self._jog(1, 0, 0)).grid(row=1, column=2, padx=3, pady=3)
        ttk.Button(btn_grid, text='Y-', width=W,
                   command=lambda: self._jog(0, -1, 0)).grid(row=2, column=1, padx=3, pady=3)

        # Z軸ジョグ（右側・独自移動距離使用）
        ttk.Separator(btn_grid, orient=tk.VERTICAL).grid(
            row=0, column=3, rowspan=3, sticky='ns', padx=6)
        ttk.Label(btn_grid, text='Z', font=('', 9, 'bold'),
                  foreground='#00897B').grid(row=0, column=4, padx=2)
        ttk.Button(btn_grid, text='Z+', width=W,
                   command=lambda: self._jog(0, 0, 1)).grid(row=1, column=4, padx=3, pady=3)
        ttk.Button(btn_grid, text='Z-', width=W,
                   command=lambda: self._jog(0, 0, -1)).grid(row=2, column=4, padx=3, pady=3)

        mf = ttk.LabelFrame(parent, text='マシン制御')
        mf.pack(fill=tk.X, padx=5, pady=(0, 5))

        ttk.Button(mf, text='🔓 アラーム解除  ($X)',
                   command=self._unlock).pack(fill=tk.X, padx=5, pady=2)
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

        # GRBLコンソールは右パネルの「コンソール表示」に統合済み

    # ------------------------------------------------------------------ file list helpers
    def _selected_entry_idx(self):
        sel = self.file_listbox.curselection()
        return sel[0] if sel else None

    def _update_file_listbox(self):
        self.file_listbox.delete(0, tk.END)
        for e in self.dxf_entries:
            self.file_listbox.insert(tk.END, f"{e['name']}  ({len(e['paths'])}パス)")
        self._update_path_listbox()

    def _update_path_listbox(self):
        """パス一覧リストボックスを再構築"""
        if not hasattr(self, 'path_listbox'):
            return
        self.path_listbox.delete(0, tk.END)
        self._path_list_data = []   # (entry, path_idx, sx, sy)
        from gcode_generator import _hierarchical_order
        for entry in self.dxf_entries:
            ox = entry.get('offset_x', 0.0)
            oy = entry.get('offset_y', 0.0)
            try:
                order, inner_flags = _hierarchical_order(entry['paths'], ox, oy)
            except Exception:
                order = list(range(len(entry['paths'])))
                inner_flags = [False]*len(entry['paths'])
            for seq, pidx in enumerate(order):
                path = entry['paths'][pidx]
                ptype = '穴' if inner_flags[pidx] else '外形'
                if path.segments:
                    sx = round(path.segments[0].start[0] + ox, 2)
                    sy = round(path.segments[0].start[1] + oy, 2)
                    label = f'{seq+1:>2}. [{ptype}] X{sx:7.2f} Y{sy:7.2f}  {entry["name"]}'
                    self.path_listbox.insert(tk.END, label)
                    self._path_list_data.append((entry, pidx, sx, sy))

    def _goto_selected_path(self):
        """選択パスの開始点へ即移動（確認なし）"""
        if not hasattr(self, 'path_listbox'):
            return
        sel = self.path_listbox.curselection()
        if not sel:
            messagebox.showinfo('未選択', 'パスを選択してください')
            return
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning('未接続', '先に接続してください')
            return
        _, _, sx, sy = self._path_list_data[sel[0]]
        self._send_serial(f'G0 X{sx:.3f} Y{sy:.3f}')
        self._log(f'>>> 🎯 G0 X{sx:.3f} Y{sy:.3f}', 'send')

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
        if ports:
            if not self.port_var.get() or self.port_var.get() not in ports:
                self.port_var.set(ports[0])
            self.connect_btn.config(state=tk.NORMAL)
        else:
            self.port_var.set('')
            # 機械未接続の場合はボタンをグレーに
            if not (self.ser and self.ser.is_open):
                self.connect_btn.config(state=tk.DISABLED)
            self.conn_status.config(text='デバイス未検出 (オフラインモード可)', foreground='#FFA500')

    def _toggle_offline(self):
        """オフラインモード: 機械なしでDXF・Gコード作業のみ行う"""
        if getattr(self, '_offline_mode', False):
            # オフライン解除
            self._offline_mode = False
            self.offline_btn.config(text='オフラインモードで使用')
            self.conn_status.config(text='未接続', foreground='gray')
            self.send_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.DISABLED)
        else:
            # オフラインON
            self._offline_mode = True
            self.offline_btn.config(text='オフライン解除')
            self.conn_status.config(
                text='オフラインモード (DXF・Gコードのみ)', foreground='#2196F3')
            # Gコード保存は使えるようにする（送信ボタンは無効のまま）
            messagebox.showinfo('オフラインモード',
                'オフラインモードで起動しました。\n\n'
                '✅ DXF読み込み\n'
                '✅ 工具経路プレビュー\n'
                '✅ Gコード保存\n'
                '❌ 機械への送信（接続時のみ）')

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
        self.connect_btn.config(text='接続中...', state=tk.DISABLED)
        self.conn_status.config(text='接続中...', foreground='#FFA500')
        threading.Thread(target=self._connect_thread, args=(port,), daemon=True).start()

    def _connect_thread(self, port):
        try:
            baud = int(self.baud_var.get())
            ser = serial.Serial(port, baud, timeout=2)
            time.sleep(2)
            ser.flushInput()
            ser.write(b'\r\n')
            time.sleep(0.3)
            resp = ser.read_all().decode('utf-8', errors='ignore').strip()

            grbl_ver = ''
            for line in resp.splitlines():
                if 'Grbl' in line or 'grbl' in line.lower():
                    grbl_ver = f'  ({line.strip()})'
                    break

            self.ser = ser
            self.polling = True
            threading.Thread(target=self._poll_thread, daemon=True).start()

            status_text = f'接続中: {port} @ {baud} bps{grbl_ver}'
            self.root.after(0, lambda: self._on_connect_success(status_text))

        except serial.SerialException as e:
            msg = str(e)
            if 'Access is denied' in msg or 'PermissionError' in msg:
                err = f'{port} にアクセスできません。\n他のアプリ（Arduino IDE等）が使用中の可能性があります。'
            elif 'could not open port' in msg.lower():
                err = f'{port} を開けません。\nデバイスが接続されているか確認してください。'
            else:
                err = msg
            self.root.after(0, lambda: self._on_connect_fail(err))
        except Exception as e:
            self.root.after(0, lambda: self._on_connect_fail(str(e)))

    def _on_connect_success(self, status_text):
        self.connect_btn.config(text='切断', state=tk.NORMAL)
        self.conn_status.config(text=status_text, foreground='green')
        self.send_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.NORMAL)
        self._log(f'✅ {status_text}', 'info')

    def _on_connect_fail(self, err_msg):
        self.ser = None
        self.connect_btn.config(text='接続', state=tk.NORMAL)
        self.conn_status.config(text='未接続', foreground='gray')
        self._log(f'❌ 接続失敗: {err_msg}', 'error')
        messagebox.showerror('接続エラー', err_msg)

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
        self._log_terminal(f'>>> {cmd.strip()}', 'send')

    def _send_realtime(self, byte: bytes):
        if not self.ser or not self.ser.is_open:
            return
        with self.serial_lock:
            self.ser.write(byte)

    # ------------------------------------------------------------------ terminal log (コンソール表示に統合)
    def _log_terminal(self, text, tag=None):
        """旧terminal → self.console に転送"""
        # タグを _log のタグ名に変換
        tag_map = {'send': 'send', 'error': 'error', 'alarm': 'error',
                   'ok': 'recv'}
        mapped = tag_map.get(tag, 'recv') if tag else 'recv'
        self.root.after(0, lambda: self._log(text, mapped))

    def _clear_terminal(self):
        self._clear_console()

    def _handle_grbl_line(self, line):
        """GRBLからの1行を解析してUI更新・ログ出力する（任意スレッドから呼び出し可）。"""
        if line.startswith('<') and line.endswith('>'):
            self._parse_position(line)
            self._parse_grbl_state(line)
        elif line.lower().startswith('ok'):
            self._log_terminal('  ok', 'ok')
        elif line.lower().startswith('error'):
            code = line.split(':')[1].strip() if ':' in line else '?'
            desc = self._grbl_error_desc(code)
            self._log_terminal(f'  ⚠ {line}  ({desc})', 'error')
        elif 'ALARM' in line.upper():
            self._log_terminal(f'  🔴 {line}', 'alarm')
        elif line:
            self._log_terminal(f'  {line}')

    def _parse_grbl_state(self, resp):
        m = re.match(r'<([^:|>]+)', resp)
        if not m:
            return
        state = m.group(1)
        state_map = {
            'Idle':  ('Idle (待機中)',          'green'),
            'Run':   ('Run (実行中)',            '#2196F3'),
            'Hold':  ('Hold (一時停止)',         '#FF9800'),
            'Alarm': ('⚠ ALARM - アラーム解除を押してください', 'red'),
            'Door':  ('Door (ドア開)',           '#FF5722'),
            'Home':  ('Home (ホーミング中)',      '#9C27B0'),
            'Jog':   ('Jog (移動中)',            '#4CAF50'),
        }
        text, color = state_map.get(state, (state, 'gray'))
        self.root.after(0, lambda t=text, c=color:
                        self.grbl_state_label.config(text=f'状態: {t}', foreground=c))

    def _grbl_error_desc(self, code):
        descs = {
            '1':  'Gコードに無効文字',
            '2':  '行頭文字が無効',
            '5':  'ホームサイクルが必要',
            '9':  'アラームロック中 → $X で解除',
            '20': 'サポート外コマンド',
            '22': 'ホームスイッチ未設定',
            '24': 'コマンド再送が必要',
        }
        return descs.get(str(code), f'コード{code}')

    # ------------------------------------------------------------------ position polling
    def _poll_thread(self):
        while self.polling and self.ser and self.ser.is_open:
            if not self.streaming:
                try:
                    with self.serial_lock:
                        # バッファに溜まった未読データを先に消化
                        pending = self.ser.in_waiting
                        if pending > 0:
                            raw = self.ser.read(pending).decode('utf-8', errors='ignore')
                            for line in raw.splitlines():
                                line = line.strip()
                                if line:
                                    self._handle_grbl_line(line)
                        # 状態ポーリング
                        self.ser.write(b'?')
                        resp = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if resp:
                        self._handle_grbl_line(resp)
                except Exception:
                    pass
            time.sleep(0.5)

    def _parse_position(self, resp):
        m = re.search(r'[MW]Pos:([-\d.]+),([-\d.]+),?([-\d.]*)', resp)
        if m:
            x, y = float(m.group(1)), float(m.group(2))
            z = float(m.group(3)) if m.group(3) else None
            self.torch_x, self.torch_y = x, y
            self.root.after(0, lambda: self._update_torch_display(x, y, z))
        if resp and not resp.startswith('<'):
            self.root.after(0, lambda r=resp: self._log(f'<<< {r}', 'recv'))

    # ------------------------------------------------------------------ helpers
    def _change_step(self, delta):
        try:
            val = float(self.jog_step.get()) + delta
            val = max(0.1, val)
            self.jog_step.set(f'{val:.0f}' if val >= 1 else f'{val:.1f}')
        except ValueError:
            self.jog_step.set('10')

    def _change_step_z(self, delta):
        try:
            val = float(self.jog_step_z.get()) + delta
            val = max(0.01, val)
            self.jog_step_z.set(f'{val:.0f}' if val >= 1 else f'{val:.2f}')
        except ValueError:
            self.jog_step_z.set('1')

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
    def _jog(self, dx, dy, dz=0):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning('警告', '先に接続してください')
            return
        step = float(self.jog_step.get())
        speed = int(float(self.jog_speed.get()))
        axis = ''
        if dx:
            axis += f' X{dx * step:.3f}'
        if dy:
            axis += f' Y{dy * step:.3f}'
        if dz:
            # Z軸は専用ステップを使用
            try:
                z_step = float(self.jog_step_z.get())
            except (ValueError, AttributeError):
                z_step = step
            axis += f' Z{dz * z_step:.3f}'
        if not axis:
            return
        self._send_serial(f'$J=G21 G91{axis} F{speed}')

    def _goto_origin(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning('警告', '先に接続してください')
            return
        self._send_serial('G90 G0 X0 Y0')

    def _unlock(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning('警告', '先に接続してください')
            return
        self._send_serial('$X')
        self.conn_status.config(text='アラーム解除済み', foreground='green')

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

        gcode = self._build_gcode(settings)
        lines = []
        for l in gcode.split('\n'):
            stripped = l.strip()
            if not stripped or stripped.startswith(';'):
                continue
            if ';' in stripped:
                stripped = stripped[:stripped.index(';')].strip()
            if stripped:
                lines.append(stripped)

        # GRBLアラーム解除 ($X) を先頭に追加
        lines = ['$X', ''] + lines
        self.streaming = True
        self.send_btn.config(state=tk.DISABLED)
        threading.Thread(target=self._stream_thread, args=(lines,), daemon=True).start()

    def _stream_thread(self, lines):
        """GRBLバッファ充填方式ストリーミング (128バイトバッファ活用)"""
        GRBL_BUF = 128
        total = len(lines)
        sent_count = 0      # 送信済み行数
        ack_count  = 0      # ok受信済み行数
        buf_used   = 0      # バッファ使用量(バイト)
        line_lens  = []     # 各行のバイト数
        error_msg  = None

        self.ser.timeout = 5  # タイムアウトを長めに

        try:
            while ack_count < total:
                if not self.streaming:
                    break

                # バッファに余裕があれば次の行を送信
                while sent_count < total:
                    line = lines[sent_count]
                    encoded = (line + '\n').encode()
                    ln = len(encoded)
                    if buf_used + ln > GRBL_BUF and line_lens:
                        break  # バッファが埋まるので待つ
                    with self.serial_lock:
                        self.ser.write(encoded)
                    line_lens.append(ln)
                    buf_used += ln
                    sent_count += 1

                # GRBLからの応答を1行読む
                with self.serial_lock:
                    resp = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if not resp:
                    continue

                if resp.lower().startswith('ok'):
                    if line_lens:
                        buf_used -= line_lens.pop(0)
                    ack_count += 1
                    pct = int(ack_count / total * 100)
                    self.root.after(0, lambda v=pct: self.progress.configure(value=v))

                elif resp.lower().startswith('error'):
                    err_line = lines[ack_count] if ack_count < total else '?'
                    error_msg = f'コマンド: {err_line}\n応答: {resp}'
                    self.root.after(0, lambda r=resp, l=err_line:
                                    self._log(f'❌ {l}  →  {r}', 'error'))
                    break

                elif resp:
                    self.root.after(0, lambda r=resp: self._log(f'<<< {r}', 'recv'))

        except Exception as e:
            error_msg = str(e)
        finally:
            self.streaming = False
            if error_msg:
                self.root.after(0, lambda m=error_msg:
                                messagebox.showerror('GRBLエラー', m))
            self.root.after(0, lambda: self.send_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.progress.configure(value=0))

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

            # ── 診断ログ & 自動配置オフセット計算 ────────────
            etypes = getattr(read_dxf, '_last_entity_types', {})
            etype_str = '  '.join(f'{k}×{v}' for k, v in sorted(etypes.items()))
            self._log(f'📂 {name}  [{etype_str}]  →  {len(paths)}パス', 'info')
            init_ox, init_oy = 0.0, 0.0
            margin = 20.0
            if len(paths) == 0:
                self._log('  ⚠ パスが0本: 未対応エンティティか空ファイルの可能性', 'error')
            else:
                all_pts = [p for path in paths for p in path.get_display_points()]
                if all_pts:
                    xs = [p[0] for p in all_pts]
                    ys = [p[1] for p in all_pts]
                    self._log(f'  座標範囲  X={min(xs):.1f}〜{max(xs):.1f}'
                              f'  Y={min(ys):.1f}〜{max(ys):.1f}', 'info')
                    # 左下をマージン位置に揃えてマシン内に自動配置
                    init_ox = round(margin - min(xs), 1)
                    init_oy = round(margin - min(ys), 1)
                else:
                    self._log('  ⚠ パスはあるが表示点ゼロ', 'error')
            # ──────────────────────────────────────────────

            self.dxf_entries.append({
                'name': name,
                'paths': paths,
                'offset_x': init_ox,
                'offset_y': init_oy,
            })
            self._cam_plan = None  # DXF変更時はCAM計画をリセット
            self._update_file_listbox()
            self.file_listbox.selection_clear(0, tk.END)
            self.file_listbox.selection_set(len(self.dxf_entries) - 1)
            self._on_file_select()
            self._draw_paths()
            self._fit_view()
            self._update_info_label()
        except Exception as e:
            self._log(f'❌ DXF読み込みエラー: {e}', 'error')
            messagebox.showerror('エラー', f'DXFの読み込みに失敗しました:\n{e}')

    def _get_settings(self):
        return {
            'feed_rate':          float(self.feed_rate.get()),
            'kerf_width':         float(self.kerf_width.get()),
            'lead_in_length':     float(self.lead_in_length.get()),
            'lead_in_type':       self.lead_in_type.get(),
            'lead_out_length':    float(self.lead_out_length.get()),
            'hot_start_distance': float(self.hot_start_distance.get()),
            'hot_start_time':     float(self.hot_start_time.get()),
            'hot_pierce_ms':      float(self.hot_pierce_ms.get()),
            'cold_pierce_ms':     float(self.cold_pierce_ms.get()),
            'rapid_speed':        float(self.rapid_speed.get()),
        }

    # ------------------------------------------------------------------ CAM editor
    def show_cam_editor(self):
        """切断順序・リードイン方向を手動編集するウィンドウ"""
        if not self.dxf_entries:
            messagebox.showwarning('警告', 'DXFファイルを先に開いてください')
            return
        CamEditorWindow(self.root, self.dxf_entries, self)

    def _build_gcode(self, settings=None):
        """CAM計画を使ってGコードを生成する（全箇所共通）"""
        if settings is None:
            settings = self._get_settings()
        plan = self.get_cam_plan()
        return generate_from_plan(plan, settings)

    def get_cam_plan(self):
        """現在の cam_plan を返す（なければ自動生成）"""
        if hasattr(self, '_cam_plan') and self._cam_plan:
            return self._cam_plan
        # 自動生成
        from gcode_generator import _hierarchical_order
        plan = []
        for entry in self.dxf_entries:
            ox = entry.get('offset_x', 0.0)
            oy = entry.get('offset_y', 0.0)
            order, inner_flags = _hierarchical_order(entry['paths'], ox, oy)
            for idx in order:
                path = entry['paths'][idx]
                pts = path.get_display_points()
                ptype = '穴' if inner_flags[idx] else '外形'
                plan.append({
                    'entry':    entry,
                    'path_idx': idx,
                    'path':     path,
                    'is_inner': inner_flags[idx],
                    'leadin':   'inside',   # 'inside' or 'outside'
                    'label':    f"{entry['name']} [{ptype}]",
                })
        self._cam_plan = plan
        return plan

    def apply_cam_plan(self, plan):
        self._cam_plan = plan

    def show_motion_preview(self):
        """トーチ動作シミュレーションウィンドウ"""
        if not self.dxf_entries:
            messagebox.showwarning('警告', 'DXFファイルを先に開いてください')
            return
        try:
            settings = self._get_settings()
            plan = self.get_cam_plan()
            gcode = self._build_gcode(settings)
        except Exception as e:
            messagebox.showerror('エラー', str(e))
            return

        # Gコードからトーチの移動リストを解析
        moves = self._parse_gcode_moves(gcode)
        if not moves:
            messagebox.showwarning('警告', '移動データがありません')
            return

        SimWindow(self.root, moves, self.dxf_entries)

    def _parse_gcode_moves(self, gcode):
        """GコードをパースしてMoveリストに変換（G2/G3アーク補間対応）
        各Move: {'x','y','rapid':bool,'torch':bool}
        """
        moves = []
        cur_x, cur_y = 0.0, 0.0
        torch_on = False

        for line in gcode.split('\n'):
            s = line.strip()
            if not s or s.startswith(';'):
                continue
            if ';' in s:
                s = s[:s.index(';')].strip()
            up = s.upper()

            if up.startswith('M3'):
                torch_on = True
                continue
            if up.startswith('M5'):
                torch_on = False
                continue

            mx = re.search(r'X([-\d.]+)', up)
            my = re.search(r'Y([-\d.]+)', up)
            mi = re.search(r'I([-\d.]+)', up)
            mj = re.search(r'J([-\d.]+)', up)

            ex = float(mx.group(1)) if mx else cur_x
            ey = float(my.group(1)) if my else cur_y

            if up.startswith('G0'):
                moves.append({'x': ex, 'y': ey, 'rapid': True, 'torch': torch_on})
                cur_x, cur_y = ex, ey

            elif up.startswith('G1'):
                moves.append({'x': ex, 'y': ey, 'rapid': False, 'torch': torch_on})
                cur_x, cur_y = ex, ey

            elif up.startswith('G2') or up.startswith('G3'):
                ccw = up.startswith('G3')
                ii = float(mi.group(1)) if mi else 0.0
                jj = float(mj.group(1)) if mj else 0.0
                acx = cur_x + ii
                acy = cur_y + jj
                r = math.hypot(cur_x - acx, cur_y - acy)
                if r > 1e-10:
                    sa = math.atan2(cur_y - acy, cur_x - acx)
                    ea = math.atan2(ey - acy, ex - acx)
                    if ccw:
                        if ea <= sa:
                            ea += 2 * math.pi
                    else:
                        if ea >= sa:
                            ea -= 2 * math.pi
                    arc_len = abs(ea - sa) * r
                    steps = max(6, int(arc_len / 1.5))
                    for k in range(1, steps + 1):
                        angle = sa + (ea - sa) * k / steps
                        moves.append({
                            'x': acx + r * math.cos(angle),
                            'y': acy + r * math.sin(angle),
                            'rapid': False, 'torch': torch_on,
                        })
                cur_x, cur_y = ex, ey

        return moves

    def show_gcode_preview(self):
        """Gコードプレビューウィンドウを表示"""
        if not self.dxf_entries:
            messagebox.showwarning('警告', 'DXFファイルを先に開いてください')
            return
        try:
            settings = self._get_settings()
            plan = self.get_cam_plan()
            gcode = self._build_gcode(settings)
        except ValueError:
            messagebox.showerror('エラー', '設定値に無効な数値があります')
            return
        except Exception as e:
            messagebox.showerror('エラー', str(e))
            return

        win = tk.Toplevel(self.root)
        win.title('Gコードプレビュー')
        win.geometry('720x620')
        win.configure(bg='#0e0e0e')

        # ── ツールバー ──
        toolbar = tk.Frame(win, bg='#1a1a2e', pady=4)
        toolbar.pack(fill=tk.X)

        lines = gcode.split('\n')
        stat = tk.Label(toolbar,
                        text=f'  行数: {len(lines)}   文字数: {len(gcode)}',
                        bg='#1a1a2e', fg='#aaccff', font=('', 9))
        stat.pack(side=tk.LEFT, padx=8)

        tk.Button(toolbar, text='💾 保存', bg='#2255cc', fg='white',
                  relief='flat', cursor='hand2', font=('', 9, 'bold'),
                  command=lambda: self._save_from_preview(gcode)
                  ).pack(side=tk.RIGHT, padx=6)
        tk.Button(toolbar, text='📋 コピー', bg='#334455', fg='white',
                  relief='flat', cursor='hand2', font=('', 9),
                  command=lambda: self._copy_to_clipboard(gcode)
                  ).pack(side=tk.RIGHT, padx=2)

        # ── 検索バー ──
        search_bar = tk.Frame(win, bg='#1a1a2e', pady=3)
        search_bar.pack(fill=tk.X)
        tk.Label(search_bar, text='🔍', bg='#1a1a2e', fg='#aaccff').pack(side=tk.LEFT, padx=6)
        search_var = tk.StringVar()
        search_entry = tk.Entry(search_bar, textvariable=search_var,
                                bg='#2a2a3e', fg='white', insertbackground='white',
                                relief='flat', font=('Consolas', 9), width=20)
        search_entry.pack(side=tk.LEFT, padx=4)
        search_result = tk.Label(search_bar, text='', bg='#1a1a2e',
                                 fg='#88ccff', font=('', 8))
        search_result.pack(side=tk.LEFT, padx=4)

        # ── テキスト + 行番号 ──
        text_frame = tk.Frame(win, bg='#0e0e0e')
        text_frame.pack(fill=tk.BOTH, expand=True, padx=4, pady=(4, 0))

        # 行番号キャンバス
        line_canvas = tk.Canvas(text_frame, width=48, bg='#1a1a1a',
                                highlightthickness=0)
        line_canvas.pack(side=tk.LEFT, fill=tk.Y)

        v_sb = tk.Scrollbar(text_frame, orient=tk.VERTICAL)
        h_sb = tk.Scrollbar(win, orient=tk.HORIZONTAL)
        v_sb.pack(side=tk.RIGHT, fill=tk.Y)

        text = tk.Text(text_frame, bg='#0e0e0e', fg='#cccccc',
                       font=('Consolas', 10), wrap='none',
                       insertbackground='white',
                       yscrollcommand=v_sb.set,
                       xscrollcommand=h_sb.set,
                       state=tk.NORMAL)
        text.pack(fill=tk.BOTH, expand=True)
        v_sb.config(command=text.yview)
        h_sb.pack(fill=tk.X, padx=4, pady=(0, 4))
        h_sb.config(command=text.xview)

        # カラータグ設定
        text.tag_config('comment', foreground='#555577')
        text.tag_config('rapid',   foreground='#ff8844')
        text.tag_config('feed',    foreground='#44cc88')
        text.tag_config('arc',     foreground='#44aaff')
        text.tag_config('torch',   foreground='#ffcc00')
        text.tag_config('coord',   foreground='#aaddff')
        text.tag_config('search',  background='#ffcc00', foreground='#000')

        # Gコードを挿入してシンタックスハイライト
        for line in lines:
            s = line.strip()
            if s.startswith(';') or not s:
                tag = 'comment'
            elif s.startswith('G0'):
                tag = 'rapid'
            elif s.startswith('G1'):
                tag = 'feed'
            elif s.startswith('G2') or s.startswith('G3'):
                tag = 'arc'
            elif s.startswith('M3') or s.startswith('M5'):
                tag = 'torch'
            else:
                tag = 'coord'
            text.insert(tk.END, line + '\n', tag)

        text.config(state=tk.DISABLED)

        # 行番号描画
        def _draw_linenos(e=None):
            line_canvas.delete('all')
            first = text.index('@0,0')
            last  = text.index(f'@0,{text.winfo_height()}')
            fl = int(first.split('.')[0])
            ll = int(last.split('.')[0])
            for n in range(fl, ll + 1):
                y = text.dlineinfo(f'{n}.0')
                if y:
                    line_canvas.create_text(
                        44, y[1] + y[3]//2,
                        text=str(n), anchor='e',
                        fill='#556677', font=('Consolas', 9))

        text.bind('<Configure>', _draw_linenos)
        text.bind('<KeyRelease>', _draw_linenos)

        def _on_yview(*args):
            text.yview(*args)
            _draw_linenos()

        v_sb.config(command=_on_yview)

        # 検索機能
        def _do_search(*_):
            text.config(state=tk.NORMAL)
            text.tag_remove('search', '1.0', tk.END)
            kw = search_var.get().strip()
            if not kw:
                search_result.config(text='')
                text.config(state=tk.DISABLED)
                return
            count = 0
            start = '1.0'
            while True:
                pos = text.search(kw, start, stopindex=tk.END)
                if not pos:
                    break
                end = f'{pos}+{len(kw)}c'
                text.tag_add('search', pos, end)
                start = end
                count += 1
                if count == 1:
                    text.see(pos)
            search_result.config(text=f'{count} 件')
            text.config(state=tk.DISABLED)

        search_var.trace_add('write', _do_search)
        win.after(100, _draw_linenos)

    def _save_from_preview(self, gcode):
        self.save_gcode(gcode_override=gcode)

    def _copy_to_clipboard(self, text):
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        messagebox.showinfo('コピー完了', 'Gコードをクリップボードにコピーしました')

    def save_gcode(self, gcode_override=None):
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
            gcode = gcode_override if gcode_override else self._build_gcode(settings)
            with open(filename, 'w') as f:
                f.write(gcode)
            messagebox.showinfo('完了', f'Gコードを保存しました:\n{filename}')
        except Exception as e:
            messagebox.showerror('エラー', f'保存に失敗しました:\n{e}')

    # ------------------------------------------------------------------ console
    def _log(self, text, tag='normal'):
        """コンソールにログを追記"""
        self.console.configure(state=tk.NORMAL)
        colors = {'send': '#88ccff', 'recv': '#00ff88', 'error': '#ff4444', 'info': '#ffcc00'}
        self.console.insert(tk.END, text + '\n', tag)
        self.console.tag_config('send',  foreground=colors['send'])
        self.console.tag_config('recv',  foreground=colors['recv'])
        self.console.tag_config('error', foreground=colors['error'])
        self.console.tag_config('info',  foreground=colors['info'])
        self.console.configure(state=tk.DISABLED)
        self.console.see(tk.END)

    def _clear_console(self):
        self.console.configure(state=tk.NORMAL)
        self.console.delete('1.0', tk.END)
        self.console.configure(state=tk.DISABLED)

    def _quick_cmd(self, cmd):
        self.manual_cmd.set(cmd)
        self._send_manual_cmd()

    def _send_manual_cmd(self):
        cmd = self.manual_cmd.get().strip()
        if not cmd:
            return
        # 履歴に追加
        if not self._cmd_hist or self._cmd_hist[-1] != cmd:
            self._cmd_hist.append(cmd)
        self._cmd_hist_idx = len(self._cmd_hist)
        self.manual_cmd.set('')
        self._log(f'>>> {cmd}', 'send')
        if not self.ser or not self.ser.is_open:
            self._log('  ⚠ 未接続', 'error')
            return
        self._send_serial(cmd)

    def _cmd_history(self, direction):
        """↑↓キーでコマンド履歴を辿る"""
        if not self._cmd_hist:
            return
        self._cmd_hist_idx = max(0, min(len(self._cmd_hist) - 1,
                                        self._cmd_hist_idx + direction))
        self.manual_cmd.set(self._cmd_hist[self._cmd_hist_idx])

    # ------------------------------------------------------------------ layout
    DEFAULT_LAYOUT = {
        'geometry': '1500x900',
        'h_sash': [820, 1040, 1260],   # h_pane の sash x 座標
        'v_sash': 315,                  # col4 の sash y 座標
    }

    def _load_layout_file(self):
        import json
        if os.path.exists(self._layout_file):
            try:
                with open(self._layout_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception:
                pass
        return {}

    def _save_layout_file(self):
        import json
        with open(self._layout_file, 'w', encoding='utf-8') as f:
            json.dump(self._layouts, f, ensure_ascii=False, indent=2)

    def _current_layout_dict(self):
        """現在のウィンドウサイズ・サッシ位置を辞書で返す"""
        geo = self.root.geometry()
        h_sash = []
        try:
            for i in range(3):
                x, _ = self._h_pane.sash_coord(i)
                h_sash.append(x)
        except Exception:
            h_sash = self.DEFAULT_LAYOUT['h_sash']
        try:
            _, y = self._col4.sash_coord(0)
            v_sash = y
        except Exception:
            v_sash = self.DEFAULT_LAYOUT['v_sash']
        return {'geometry': geo, 'h_sash': h_sash, 'v_sash': v_sash}

    def _apply_layout_dict(self, d):
        """辞書からレイアウトを適用する"""
        try:
            self.root.geometry(d.get('geometry', '1500x900'))
            self.root.update_idletasks()
            for i, x in enumerate(d.get('h_sash', [])):
                self._h_pane.sash_place(i, x, 0)
            self._col4.sash_place(0, 0, d.get('v_sash', 315))
        except Exception as e:
            print('レイアウト適用エラー:', e)

    def _apply_default_layout(self):
        self._apply_layout_dict(self.DEFAULT_LAYOUT)

    def _save_layout_as(self):
        """名前を付けてレイアウトを保存"""
        from tkinter.simpledialog import askstring
        name = askstring('レイアウト保存', 'レイアウト名を入力してください:',
                         parent=self.root)
        if not name or not name.strip():
            return
        name = name.strip()
        self._layouts[name] = self._current_layout_dict()
        self._layouts['__last__'] = self._current_layout_dict()
        self._save_layout_file()
        self._rebuild_layout_menu()
        messagebox.showinfo('保存完了', f'レイアウト「{name}」を保存しました。')

    def _load_layout(self, name):
        d = self._layouts.get(name)
        if d:
            self._apply_layout_dict(d)
            self._layouts['__last__'] = d
            self._save_layout_file()

    def _delete_layout(self, name):
        if name in self._layouts:
            if messagebox.askyesno('確認', f'「{name}」を削除しますか？'):
                del self._layouts[name]
                self._save_layout_file()
                self._rebuild_layout_menu()

    def _rebuild_layout_menu(self):
        """保存済みレイアウト一覧をメニューに反映"""
        # 固定メニュー2項目 + separator の後をクリア
        end = self._layout_menu.index('end')
        if end is not None and end >= 3:
            for _ in range(end - 2):
                self._layout_menu.delete(3)

        saved = [k for k in self._layouts if not k.startswith('__')]
        if saved:
            for name in saved:
                sub = tk.Menu(self._layout_menu, tearoff=0)
                sub.add_command(label='適用',
                                command=lambda n=name: self._load_layout(n))
                sub.add_command(label='削除',
                                command=lambda n=name: self._delete_layout(n))
                self._layout_menu.add_cascade(label=f'  {name}', menu=sub)

    def _on_close(self):
        self.streaming = False
        self.polling = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        # 終了時に設定を自動保存
        try:
            raw = {k: str(v) for k, v in self._get_settings().items()}
            settings_manager.save(raw, self._presets)
        except Exception:
            pass
        # 終了時にレイアウトを自動保存
        try:
            self._layouts['__last__'] = self._current_layout_dict()
            self._save_layout_file()
        except Exception:
            pass
        self.root.destroy()


# ======================================================================
# SimWindow: トーチ動作シミュレーション (tkinter Canvas版)
# ======================================================================
class SimWindow:
    SPEEDS = [('×0.25', 0.25), ('×0.5', 0.5), ('×1', 1.0),
              ('×2', 2.0), ('×5', 5.0), ('×10', 10.0), ('×50', 50.0)]
    INTERVAL_MS = 20      # タイマー間隔(ms) — 固定
    MM_PER_TICK_CUT = 1.5 # 1tickで進む距離(切削)
    MM_PER_TICK_RAP = 6.0 # 1tickで進む距離(早送り)

    def __init__(self, parent, moves, dxf_entries):
        self.moves   = moves
        self.entries = dxf_entries
        self._playing  = False
        self._speed    = 1.0
        self._after_id = None
        self._target   = 0
        self._cur_x    = moves[0]['x'] if moves else 0.0
        self._cur_y    = moves[0]['y'] if moves else 0.0

        # 座標範囲を計算
        xs = [m['x'] for m in moves]
        ys = [m['y'] for m in moves]
        self._xmin = min(xs); self._xmax = max(xs)
        self._ymin = min(ys); self._ymax = max(ys)

        self._build(parent)
        self._reset()

    # ── 座標変換 ──────────────────────────────────────────
    def _to_canvas(self, x, y):
        W = self._cv.winfo_width()  or 800
        H = self._cv.winfo_height() or 600
        pad = 40
        rng_x = max(self._xmax - self._xmin, 1)
        rng_y = max(self._ymax - self._ymin, 1)
        scale = min((W - pad*2) / rng_x, (H - pad*2) / rng_y)
        cx = pad + (x - self._xmin) * scale
        cy = H - pad - (y - self._ymin) * scale   # Y軸反転
        return cx, cy

    # ── UI構築 ──────────────────────────────────────────
    def _build(self, parent):
        win = tk.Toplevel(parent)
        win.title('▶ 動作プレビュー')
        win.geometry('920x700')
        win.configure(bg='#0a0a12')
        self._win = win
        win.protocol('WM_DELETE_WINDOW', self._on_close)

        # ── ツールバー ──
        tb = tk.Frame(win, bg='#12122a', pady=6)
        tb.pack(fill=tk.X)

        self._play_btn = tk.Button(
            tb, text='▶  再生', bg='#1144bb', fg='white',
            font=('Yu Gothic UI', 11, 'bold'), relief='flat',
            cursor='hand2', padx=14, command=self._toggle_play)
        self._play_btn.pack(side=tk.LEFT, padx=8)

        tk.Button(tb, text='⏹ リセット', bg='#223344', fg='#aaccee',
                  font=('Yu Gothic UI', 9), relief='flat',
                  cursor='hand2', padx=10,
                  command=self._reset).pack(side=tk.LEFT, padx=4)

        tk.Label(tb, text='速度:', bg='#12122a',
                 fg='#8899bb', font=('', 9)).pack(side=tk.LEFT, padx=(16,2))
        self._speed_var = tk.StringVar(value='×1')
        speed_cb = ttk.Combobox(tb, textvariable=self._speed_var,
                                values=[s[0] for s in self.SPEEDS],
                                width=6, state='readonly')
        speed_cb.pack(side=tk.LEFT)
        speed_cb.bind('<<ComboboxSelected>>', self._on_speed)

        self._info_var = tk.StringVar(value='▶ を押して開始')
        tk.Label(tb, textvariable=self._info_var, bg='#12122a',
                 fg='#88ccff', font=('Consolas', 9)).pack(side=tk.LEFT, padx=16)

        self._torch_var = tk.StringVar(value='● トーチOFF')
        self._torch_lbl = tk.Label(tb, textvariable=self._torch_var,
                                   bg='#12122a', fg='#ff4444',
                                   font=('Yu Gothic UI', 10, 'bold'))
        self._torch_lbl.pack(side=tk.RIGHT, padx=14)

        # ── 凡例 ──
        leg = tk.Frame(win, bg='#0a0a12')
        leg.pack(fill=tk.X, padx=6, pady=2)
        for col, lbl in [('#ff8833','早送り'), ('#44ff88','切削'),
                          ('#ffff00','トーチ位置')]:
            tk.Frame(leg, bg=col, width=20, height=4).pack(
                side=tk.LEFT, padx=(8,2))
            tk.Label(leg, text=lbl, bg='#0a0a12', fg='#778899',
                     font=('',8)).pack(side=tk.LEFT, padx=(0,12))

        # ── プログレスバー ──
        self._prog = ttk.Progressbar(win, mode='determinate')
        self._prog.pack(fill=tk.X, padx=8, pady=(0,2))

        # ── Canvas ──
        self._cv = tk.Canvas(win, bg='#050508',
                             highlightthickness=0)
        self._cv.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        self._cv.bind('<Configure>', lambda e: self._redraw_all())

    # ── リセット ────────────────────────────────────────
    def _reset(self):
        self._stop()
        self._target = 0
        if self.moves:
            self._cur_x = self.moves[0]['x']
            self._cur_y = self.moves[0]['y']
        self._prog['value'] = 0
        self._info_var.set('▶ を押して開始')
        self._torch_var.set('● トーチOFF')
        self._torch_lbl.config(fg='#ff4444')
        self._play_btn.config(text='▶  再生')
        self._redraw_all()

    def _redraw_all(self):
        """キャンバスをクリアしてDXFジオメトリを再描画"""
        self._cv.delete('all')
        # グリッド
        W = self._cv.winfo_width()  or 800
        H = self._cv.winfo_height() or 600
        for i in range(0, W, 50):
            self._cv.create_line(i, 0, i, H, fill='#111122', width=1)
        for i in range(0, H, 50):
            self._cv.create_line(0, i, W, i, fill='#111122', width=1)
        # DXFジオメトリ
        for entry in self.entries:
            ox, oy = entry['offset_x'], entry['offset_y']
            for path in entry['paths']:
                pts = path.get_display_points()
                if len(pts) < 2:
                    continue
                coords = []
                for p in pts:
                    cx, cy = self._to_canvas(p[0]+ox, p[1]+oy)
                    coords += [cx, cy]
                self._cv.create_line(*coords, fill='#223355',
                                     width=1, tags='geo')

    # ── 再生制御 ────────────────────────────────────────
    def _toggle_play(self):
        if self._playing:
            self._stop()
        else:
            if self._target >= len(self.moves):
                self._reset()
            self._play()

    def _play(self):
        self._playing = True
        self._play_btn.config(text='⏸  一時停止')
        self._tick()

    def _stop(self):
        self._playing = False
        self._play_btn.config(text='▶  再生')
        if self._after_id:
            try:
                self._win.after_cancel(self._after_id)
            except Exception:
                pass
            self._after_id = None

    def _on_speed(self, e=None):
        sel = self._speed_var.get()
        for lbl, val in self.SPEEDS:
            if lbl == sel:
                self._speed = val

    # ── アニメーション本体 ──────────────────────────────
    def _tick(self):
        if not self._playing:
            return
        if self._target >= len(self.moves):
            self._stop()
            self._info_var.set('✅ 完了！')
            self._prog['value'] = 100
            return

        move    = self.moves[self._target]
        tx, ty  = move['x'], move['y']
        rapid   = move['rapid']
        torch   = move['torch']

        mm_tick = (self.MM_PER_TICK_RAP if rapid
                   else self.MM_PER_TICK_CUT) * self._speed
        dx = tx - self._cur_x
        dy = ty - self._cur_y
        dist = math.hypot(dx, dy)

        if dist < 0.5:
            self._cur_x, self._cur_y = tx, ty
            self._target += 1
            self._after_id = self._win.after(1, self._tick)
            return

        t  = min(mm_tick / dist, 1.0)
        nx = self._cur_x + dx * t
        ny = self._cur_y + dy * t

        # 線を描く
        x1, y1 = self._to_canvas(self._cur_x, self._cur_y)
        x2, y2 = self._to_canvas(nx, ny)
        color  = '#ff8833' if rapid else '#44ff88'
        dash   = (4, 4) if rapid else None
        width  = 1 if rapid else 2
        self._cv.create_line(x1, y1, x2, y2,
                             fill=color, width=width,
                             dash=dash, tags='trace')

        # トーチマーカー
        self._cv.delete('torch')
        r = 6
        tc = '#ffff00' if torch else '#ffffff'
        self._cv.create_oval(x2-r, y2-r, x2+r, y2+r,
                             fill=tc, outline='white',
                             width=1, tags='torch')
        # 切削中の火花
        if torch and not rapid:
            import random
            for _ in range(4):
                ang = random.uniform(0, 2*math.pi)
                ln  = random.uniform(4, 12)
                sx  = x2 + math.cos(ang)*ln
                sy  = y2 + math.sin(ang)*ln
                sid = self._cv.create_line(
                    x2, y2, sx, sy,
                    fill=random.choice(['#ffaa00','#ffcc44','#ffffff']),
                    width=1, tags='spark')
                self._win.after(60, lambda i=sid: self._cv.delete(i))

        self._cur_x, self._cur_y = nx, ny

        # UI更新
        pct = int(self._target / len(self.moves) * 100)
        self._prog['value'] = pct
        self._info_var.set(
            f'{self._target}/{len(self.moves)}  '
            f'X={nx:.1f}  Y={ny:.1f}')
        if torch:
            self._torch_var.set('🔥 トーチON（切断中）')
            self._torch_lbl.config(fg='#ffcc00')
        else:
            self._torch_var.set('● トーチOFF')
            self._torch_lbl.config(fg='#ff4444')

        self._after_id = self._win.after(self.INTERVAL_MS, self._tick)

    def _on_close(self):
        self._stop()
        self._win.destroy()


# ======================================================================
# CamEditorWindow: 切断順序・リードイン方向 手動編集
# ======================================================================
class CamEditorWindow:
    def __init__(self, parent, dxf_entries, app):
        self.app = app
        self.plan = app.get_cam_plan()[:]   # コピー

        win = tk.Toplevel(parent)
        win.title('⚙ CAM編集 - 切断順序 / リードイン方向')
        win.geometry('700x520')
        self._win = win
        win.protocol('WM_DELETE_WINDOW', self._on_close_editor)

        # ── 説明 ──
        tk.Label(win,
                 text='行をクリック → キャンバスで場所を確認  ／  ↑↓で順序変更  ／  ダブルクリックでリードイン切替',
                 font=('Yu Gothic UI', 9), fg='#555').pack(pady=(8, 2))

        # ── テーブル ──
        frame = ttk.Frame(win)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)

        cols = ('order', 'type', 'leadin', 'name')
        self._tv = ttk.Treeview(frame, columns=cols, show='headings',
                                selectmode='browse', height=16)
        self._tv.heading('order',  text='順番')
        self._tv.heading('type',   text='種類')
        self._tv.heading('leadin', text='リードイン方向')
        self._tv.heading('name',   text='パス名')
        self._tv.column('order',  width=55,  anchor='center')
        self._tv.column('type',   width=70,  anchor='center')
        self._tv.column('leadin', width=130, anchor='center')
        self._tv.column('name',   width=380, anchor='w')

        sb = ttk.Scrollbar(frame, command=self._tv.yview)
        self._tv.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self._tv.pack(fill=tk.BOTH, expand=True)
        self._tv.bind('<Double-1>',          self._on_double_click)
        self._tv.bind('<<TreeviewSelect>>', self._on_select)
        self._tv.tag_configure('outer',    background='#fff3e0')
        self._tv.tag_configure('inner',    background='#e8f5e9')
        self._tv.tag_configure('selected', background='#fffacd')

        # ── ボタン行 ──
        btn_frame = ttk.Frame(win)
        btn_frame.pack(fill=tk.X, padx=10, pady=6)

        ttk.Button(btn_frame, text='⬆ 上へ',
                   command=self._move_up).pack(side=tk.LEFT, padx=3)
        ttk.Button(btn_frame, text='⬇ 下へ',
                   command=self._move_down).pack(side=tk.LEFT, padx=3)

        ttk.Separator(btn_frame, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Button(btn_frame, text='🔄 リードイン切替 (内側↔外側)',
                   command=self._toggle_leadin).pack(side=tk.LEFT, padx=3)

        ttk.Separator(btn_frame, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Button(btn_frame, text='🎯 ここへ移動',
                   command=self._goto_selected).pack(side=tk.LEFT, padx=3)

        ttk.Separator(btn_frame, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Button(btn_frame, text='🔃 自動順序に戻す',
                   command=self._auto_order).pack(side=tk.LEFT, padx=3)

        ttk.Button(btn_frame, text='✅ 適用して閉じる',
                   command=self._apply).pack(side=tk.RIGHT, padx=3)
        ttk.Button(btn_frame, text='キャンセル',
                   command=win.destroy).pack(side=tk.RIGHT, padx=3)

        # ── 凡例 ──
        leg = ttk.Frame(win)
        leg.pack(fill=tk.X, padx=10, pady=(0, 6))
        tk.Frame(leg, bg='#e8f5e9', width=16, height=12).pack(side=tk.LEFT, padx=(0,3))
        tk.Label(leg, text='穴(内径カット)', font=('',8)).pack(side=tk.LEFT, padx=(0,12))
        tk.Frame(leg, bg='#fff3e0', width=16, height=12).pack(side=tk.LEFT, padx=(0,3))
        tk.Label(leg, text='外形カット', font=('',8)).pack(side=tk.LEFT)
        tk.Label(leg,
                 text='  ※ ダブルクリックでリードイン方向を切替',
                 font=('', 8), fg='#888').pack(side=tk.RIGHT)

        self._refresh()

    def _refresh(self):
        self._tv.delete(*self._tv.get_children())
        for i, item in enumerate(self.plan):
            is_inner = item['is_inner']
            leadin   = item.get('leadin', 'inside')
            ptype    = '穴' if is_inner else '外形'
            ldlabel  = '← 内側から' if leadin == 'inside' else '→ 外側から'
            tag      = 'inner' if is_inner else 'outer'
            self._tv.insert('', 'end',
                            iid=str(i),
                            values=(i+1, ptype, ldlabel, item['label']),
                            tags=(tag,))

    def _selected_idx(self):
        sel = self._tv.selection()
        return int(sel[0]) if sel else None

    def _move_up(self):
        idx = self._selected_idx()
        if idx is None or idx == 0:
            return
        self.plan[idx-1], self.plan[idx] = self.plan[idx], self.plan[idx-1]
        self._refresh()
        self._tv.selection_set(str(idx-1))
        self._tv.see(str(idx-1))

    def _move_down(self):
        idx = self._selected_idx()
        if idx is None or idx >= len(self.plan)-1:
            return
        self.plan[idx], self.plan[idx+1] = self.plan[idx+1], self.plan[idx]
        self._refresh()
        self._tv.selection_set(str(idx+1))
        self._tv.see(str(idx+1))

    def _toggle_leadin(self):
        idx = self._selected_idx()
        if idx is None:
            return
        cur = self.plan[idx].get('leadin', 'inside')
        self.plan[idx]['leadin'] = 'outside' if cur == 'inside' else 'inside'
        self._refresh()
        self._tv.selection_set(str(idx))

    def _on_select(self, e=None):
        """行選択 → メインキャンバスでパスをハイライト"""
        idx = self._selected_idx()
        if idx is None:
            self.app.clear_highlight()
            return
        item = self.plan[idx]
        self.app.highlight_path(item['entry'], item['path_idx'])

    def _goto_selected(self):
        """選択パスの開始点へマシンを移動（G0）"""
        idx = self._selected_idx()
        if idx is None:
            messagebox.showwarning('未選択', 'パスを選択してください', parent=self._win)
            return

        # シリアル接続確認
        if not self.app.ser or not self.app.ser.is_open:
            messagebox.showwarning('未接続',
                                   '先にシリアル接続してください\n'
                                   '（設定タブ → 接続ボタン）',
                                   parent=self._win)
            return

        item = self.plan[idx]
        entry    = item['entry']
        path     = item['path']
        ox = entry.get('offset_x', 0.0)
        oy = entry.get('offset_y', 0.0)

        # パスの開始点を取得
        if path.segments:
            sx = path.segments[0].start[0] + ox
            sy = path.segments[0].start[1] + oy
        else:
            messagebox.showwarning('エラー', 'パスに座標がありません', parent=self._win)
            return

        msg = (f'トーチを以下の位置に移動します:\n\n'
               f'  X = {sx:.3f} mm\n'
               f'  Y = {sy:.3f} mm\n\n'
               f'よろしいですか？')
        if not messagebox.askyesno('移動確認', msg, parent=self._win):
            return

        safe_z = float(self.app._get_settings().get('safe_z', 5.0)) \
            if hasattr(self.app, '_get_settings') else 5.0

        # 安全高さに上げてから移動
        self.app._send_serial(f'G0 Z{safe_z:.3f}')
        self.app._send_serial(f'G0 X{sx:.3f} Y{sy:.3f}')
        self.app._log(f'>>> 🎯 G0 X{sx:.3f} Y{sy:.3f}  (パス{idx+1} 開始点)', 'send')

    def _on_double_click(self, e):
        self._toggle_leadin()

    def _auto_order(self):
        from gcode_generator import _hierarchical_order
        new_plan = []
        for entry in self.app.dxf_entries:
            ox = entry.get('offset_x', 0.0)
            oy = entry.get('offset_y', 0.0)
            order, inner_flags = _hierarchical_order(entry['paths'], ox, oy)
            for idx in order:
                path  = entry['paths'][idx]
                ptype = '穴' if inner_flags[idx] else '外形'
                # 既存の leadin 設定を引き継ぐ
                old = next((p for p in self.plan
                            if p['entry'] is entry and p['path_idx'] == idx), None)
                new_plan.append({
                    'entry':    entry,
                    'path_idx': idx,
                    'path':     path,
                    'is_inner': inner_flags[idx],
                    'leadin':   old['leadin'] if old else 'inside',
                    'label':    f"{entry['name']} [{ptype}]",
                })
        self.plan = new_plan
        self._refresh()

    def _apply(self):
        self.app.apply_cam_plan(self.plan)
        self.app.clear_highlight()
        messagebox.showinfo('適用完了',
                            f'{len(self.plan)}パスのCAM計画を適用しました。\n'
                            '「▶ プレビュー」または「💾 保存」で確認できます。')
        self._win.destroy()

    def _on_close_editor(self):
        self.app.clear_highlight()
        self._win.destroy()


if __name__ == '__main__':
    root = tk.Tk()
    app = PlasmaCamApp(root)
    root.mainloop()
