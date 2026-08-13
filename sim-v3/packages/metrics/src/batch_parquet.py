from __future__ import annotations
import pathlib,sys
import pyarrow.json as pajson
import pyarrow.parquet as parquet
root=pathlib.Path(sys.argv[1])
count=0
for source in sorted(root.rglob("*.jsonl")):
    if source.stat().st_size==0: continue
    table=pajson.read_json(source,read_options=pajson.ReadOptions(block_size=1<<20))
    parquet.write_table(table,source.with_suffix(".parquet"),row_group_size=2048,compression="zstd")
    count+=1
print(count)
