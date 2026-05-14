import multiprocessing as mp
import os, sys

if mp.current_process().name != 'MainProcess':
    # Child!
    def fake(path):
        class Dummy:
            def close(self): pass
        print(f"Blocked os.add_dll_directory({path})")
        return Dummy()
    os.add_dll_directory = fake

from PyQt5.QtWidgets import QApplication

def worker(q):
    try:
        import torch
        q.put('torch OK')
    except Exception as e:
        q.put(str(e))

if __name__ == '__main__':
    ctx = mp.get_context('spawn')
    q = ctx.Queue()
    p = ctx.Process(target=worker, args=(q,))
    p.start()
    print(q.get())
    p.join()
