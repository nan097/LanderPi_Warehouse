from pathlib import Path
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit(): continue
    try:
        args=(proc/'cmdline').read_bytes().split(b'\0')
        if args and Path(args[0].decode()).name in ['gzserver','controller_server','sort_runtime.py','bt_navigator']:
            print(proc.name, b' '.join(args).decode()[:250]); print([v.decode() for v in (proc/'environ').read_bytes().split(b'\0') if v.startswith((b'ROS_DOMAIN_ID=', b'GAZEBO_MASTER_URI=', b'FASTRTPS_DEFAULT_PROFILES_FILE='))])
    except (OSError,UnicodeError): pass
