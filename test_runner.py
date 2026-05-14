import sys, os
from test_spawn import inference_worker_test

if __name__ == '__main__':
    with open('debug_spawn.log', 'w') as f:
        f.write('Start\n')

    from PyQt5.QtWidgets import QApplication
    app = QApplication(sys.argv)
    
    import multiprocessing as mp
    ctx = mp.get_context('spawn')
    q_in = ctx.Queue()
    q_out = ctx.Queue()
    p = ctx.Process(target=inference_worker_test, args=(q_in, q_out))
    
    # Simulate cleaning PATH
    import pathlib
    _torch_lib = (pathlib.Path(sys.executable).parent.parent /
                  "Lib" / "site-packages" / "torch" / "lib")
    _orig_path = os.environ.get("PATH", "")
    _clean_parts = []
    for _p in _orig_path.split(os.pathsep):
        _pl = _p.lower()
        if ("cuda" in _pl and "nvidia gpu computing toolkit" in _pl) or \
           "pyqt5" in _pl or "\\qt\\" in _pl:
            continue
        _clean_parts.append(_p)
    if _torch_lib.is_dir():
        _clean_parts.insert(0, str(_torch_lib))
    os.environ["PATH"] = os.pathsep.join(_clean_parts)
    
    p.start()
    
    os.environ["PATH"] = _orig_path
    
    res = q_out.get()
    with open('debug_spawn.log', 'a') as f:
        f.write(f'Main received: {res}\n')
    p.join()
