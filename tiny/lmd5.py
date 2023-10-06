import os
import hashlib
import datetime
# assign directory
directory = '.'

print(datetime.datetime.today())
 
# iterate over files in
# that directory
def tra(directory):
  for filename in os.listdir(directory):
    f = os.path.join(directory, filename)
    # checking if it is a file
    if os.path.isfile(f):
      with open(f,"rb") as fl:
        md5_hash = hashlib.md5()
        for byte_block in iter(lambda: fl.read(2097152),b""):
          md5_hash.update(byte_block)
        hv = md5_hash.hexdigest();        
        print(hv,end=" ")
      print(f)
    else:
      tra(f)
tra(directory)
print(datetime.datetime.today())
