"""main script

This script uses multiprocessing to spawn new processes for each file specified allowing
them to run simultaneously.

adjust the sleep timer if the script quits before the subprocesses are complete
need to fix this...

"""

import multiprocessing
import subprocess
import time
import logging

# https://docs.python.org/3.7/library/multiprocessing.html#multiprocessing.Process.run
# https://docs.python.org/3/library/subprocess.html
# https://stackoverflow.com/questions/46272718/listening-from-multiple-udp-sockets-in-python-script

logging.basicConfig(level=logging.DEBUG)

# worker function
# using subprocess runs the python file provided in the argument
def worker(file, idNum):
    name = multiprocessing.current_process().name               # retreive the name of the process
    name = name + '_' + str(i+1)                                # assign a unique name
    logging.debug(' ' + name + ' Starting')
    subprocess.Popen(['python', file])                          # execute the file in a new process (Popen)
    time.sleep(1)                                               # ensures one process starts before the other
    logging.debug(' ' + name + ' Exiting')


if __name__ == '__main__':

    time_start = time.time()                                        # for debugging
    
    files = ['obj_server.py', 'image_client.py', 'controller_bob.py', 'controller_bobette.py']   # list of files to run
    try:
        for i, file in enumerate(files):                            # for each file >> apply multiprocessing.Process on the worker function
            p = multiprocessing.Process(target=worker(file, i))     # basically allowing both files to run at the same time on seperate processes
            p.start()                                               # start the process p
        p.join()                                                    # blocks until p is complete (ensures both child processes complete)
        p.close()                                                   # make sure to close process obj

        time.sleep(2000000.0)                                           # i think this needs to be as long as u need it for the script to continue indefinitely
        
        print(round((time.time()-time_start), ndigits=2))           # for debugging

    except KeyboardInterrupt:                                       # hit Ctrl+c to force quit
        logging.debug('Quitting [mp_main.py] from keypress [ctrl+c]')
        pass
