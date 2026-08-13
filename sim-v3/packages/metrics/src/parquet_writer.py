from __future__ import annotations
import json
import os
import pathlib
import sys
import pyarrow as pa
import pyarrow.parquet as pq

source=pathlib.Path(sys.argv[1]);target=pathlib.Path(sys.argv[2]);batch_size=int(sys.argv[3]) if len(sys.argv)>3 else 2048
target.parent.mkdir(parents=True,exist_ok=True);temporary=target.with_suffix(target.suffix+".tmp");rows=[];count=0;groups=0;writer=None;schema=None
try:
    with source.open() as stream:
        for line in stream:
            if not line.strip(): continue
            rows.append(json.loads(line))
            if len(rows)>=batch_size:
                table=pa.Table.from_pylist(rows,schema=schema);schema=table.schema;writer=writer or pq.ParquetWriter(temporary,schema,compression="zstd");writer.write_table(table);count+=len(rows);groups+=1;rows=[]
        if rows:
            table=pa.Table.from_pylist(rows,schema=schema);schema=table.schema;writer=writer or pq.ParquetWriter(temporary,schema,compression="zstd");writer.write_table(table);count+=len(rows);groups+=1
    if writer is None: raise SystemExit("No rows supplied for Parquet output")
finally:
    if writer is not None: writer.close()
os.replace(temporary,target)
print(json.dumps({"rows":count,"row_groups":groups,"batch_size":batch_size,"bytes":target.stat().st_size,"path":str(target),"schema":str(schema)}))
