
Python wrapper for sqlite3
==========


python script reading and writing data to sqlite3 database specially designed for __time-stamped data__. 




Key Info
--------------------------------------

Classes for reading & writing timestamped data to the sqlite3 database via python

- Dependencies are sqlite3, pandas and datetime
- Writer automatically keep track of timestamps. Timestamp are used as primary key
- Writer automatically handle creation and destruction of tables (columns)
- Writer automatically delete samples older than threshold and empty tables (columns)
- Input to writer are four lists of equal size; tag,datatype,value & metadata
- Reader return a pandas dataframe
- Reader can extract every sample or a subset of the samples based on time-intervals
- Multiple reader instances can read from the same database simultaniously


Example Usage
---------

__Writing to database__
- *deltaDays*, *hours*,  *minutes* & *seconds* determine how long samples are stored before deleted
- Valid datatypes are _float_, _integer_, _text_, _null_ and _blob_ (Refer to sqlite3 documentation)
- *handshake* set to true will cause SQLite to wait on data to reach the disk surface before moving on. This will cause writing of data into database to take longer time, but are more stable.
- If you lose power in the middle of a transaction and *handshake* are set to *False*, your database file might go corrupt




```
from Git.SqliteWrapper.sqliteWrapper import SQLiteReader, SQLiteWriter
import time
import pandas
import matplotlib.pyplot as plt
import datetime
import numpy as np

writer = SQLiteWriter(file='database.db',handshake=False)
writer.deltaDays = 0
writer.deltaHours = 0
writer.deltaMinutes = 0
writer.deltaSeconds = 1

tag = ['pressure','temperature','flow']
datatype = ['float','float','float']
metadata = ['Healthy','Alarm','Sensor Failure']

for i in range(10):
    value = np.random.uniform(low=0,high=1,size=(3))
    writer.write(tag=tag, value=value, datatype=datatype, metadata=metadata, deleteOldSamples=True)
    time.sleep(0.1)
```

__Reading from database__
- start and stop are datetime objects. If they are set to None then all samples are read
- tables are list of columns to be read. If it are set to None then all columns are read
- *metadata* are set to *True* of *False* depending on whether columns of metadata shall be included in response
- *sampletime* are a *datetime.timedelta* object determine time between each extracted sample
- If *sampletime* are set to *None* then all samples are read


```
reader = SQLiteReader(file='database.db')
dataFrame = reader.read(tables=None,start=None,stop=None,metadata=True,sampletime=None)

print("Tables: " + str(reader.getTables()))
print("Number of rows: " + str(reader.getNumberofRows(table=reader.getTables()[0])))
print("...")
print(dataFrame)

dataFrame.to_excel("database.xlsx")
dataFrame.to_csv("database.csv")

dataFrame = dataFrame.to_dict()
pressure_timestamp = list(dataFrame['pressure'].keys())
pressure_value = list(dataFrame['pressure'].values())
pressure_metadata = list(dataFrame['pressure Metadata'].values())

plt.plot(pressure_timestamp,pressure_value)
plt.grid(True)
plt.show()
```






