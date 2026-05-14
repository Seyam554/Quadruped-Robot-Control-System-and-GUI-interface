import os, sys, traceback

def inference_worker_test(q_in, q_out):
    try:
        with open('debug_spawn.log', 'a') as f:
            f.write(f'Worker started! PID={os.getpid()}\n')
            p = os.environ.get("PATH")
            f.write(f'PATH={p}\n')
        
        try:
            import torch
            with open('debug_spawn.log', 'a') as f:
                f.write('Imports successful.\n')
        except Exception as e:
            with open('debug_spawn.log', 'a') as f:
                f.write(f'IMPORT ERROR: {e}\n')
                f.write(traceback.format_exc())
            
        q_out.put(('ready', None))
    except Exception as exc:
        with open('debug_spawn.log', 'a') as f:
            f.write(f'OUTER ERROR: {exc}\n')
        q_out.put(('error', str(exc)))
