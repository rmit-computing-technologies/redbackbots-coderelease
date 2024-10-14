"""
Python module to restart pulseaudio + dbus if the amixer program reports
the wrong number of capture channels online.
"""
import subprocess
import time

# To test this, change MAGIC_CAPTURE_COUNT to another number and "nao restart".
# Then change it back to 7 and again "nao restart" (nao_sync-ing if necessary).
#
# I don't like the magic number, but am unsure how else to detect that we have
# all 4 'Capture' channels online instead of just 2. See #740 for details.
# MAGIC_CAPTURE_COUNT = 7
MAGIC_CAPTURE_COUNT = 20


start = time.time()
cmd = subprocess.Popen(
    ['amixer'],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
)
cmd_out = cmd.communicate()[0]
print(cmd_out)
if cmd_out.count('Capture'.encode('utf-8')) != MAGIC_CAPTURE_COUNT:
    print('WHISTLES WILL NOT BE HEARD!')
    
    taken = time.time() - start

    # Construct a non-zero exit code
    exit(int(taken) % 254 + 1)

exit(0)
