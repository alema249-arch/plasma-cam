import json, os

SETTINGS_FILE = os.path.join(os.path.dirname(__file__), 'pierce_settings.json')

DEFAULT_SETTINGS = {
    'feed_rate':           '3000',
    'kerf_width':          '1.5',
    'lead_in_length':      '5.0',
    'lead_in_type':        'line',
    'lead_out_length':     '3.0',
    'pierce_delay':        '0.5',
    'post_cut_delay':      '0.0',
    'pierce_z_height':     '0.5',
    'cut_z_height':        '1.5',
    'home_z_clearance':    '10.0',
    'hot_start_distance':  '50.0',
    'hot_start_time':      '2.5',
    'hot_pierce_ms':       '800',
    'cold_pierce_ms':      '2400',
    'rapid_speed':         '5000',
}

DEFAULT_PRESETS = {
    '標準': {'hot_start_distance':'50.0','hot_start_time':'2.5',
             'hot_pierce_ms':'800','cold_pierce_ms':'2400'},
    '夏用': {'hot_start_distance':'60.0','hot_start_time':'3.0',
             'hot_pierce_ms':'600','cold_pierce_ms':'2000'},
    '冬用': {'hot_start_distance':'40.0','hot_start_time':'2.0',
             'hot_pierce_ms':'1000','cold_pierce_ms':'3000'},
}

def load():
    if not os.path.exists(SETTINGS_FILE):
        return dict(DEFAULT_SETTINGS), dict(DEFAULT_PRESETS)
    try:
        with open(SETTINGS_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
        settings = {**DEFAULT_SETTINGS, **data.get('settings', {})}
        presets  = {**DEFAULT_PRESETS,  **data.get('presets',  {})}
        return settings, presets
    except Exception:
        return dict(DEFAULT_SETTINGS), dict(DEFAULT_PRESETS)

def save(settings: dict, presets: dict):
    try:
        with open(SETTINGS_FILE, 'w', encoding='utf-8') as f:
            json.dump({'settings': settings, 'presets': presets},
                      f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f'設定保存失敗: {e}')
