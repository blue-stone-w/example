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
        bytes = fl.read()
        hv = hashlib.md5(bytes).hexdigest();        
        print(hv,end=" ")
      print(f)
    else:
      tra(f)
tra(directory)
print(datetime.datetime.today())
