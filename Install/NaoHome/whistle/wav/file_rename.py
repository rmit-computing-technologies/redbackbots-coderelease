import os

files = os.listdir(os.curdir)
for file in files:
    os.rename(file, file.replace('_', ''))