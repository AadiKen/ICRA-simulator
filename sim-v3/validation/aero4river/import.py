"""Import checksum-locked AERO4River MATLAB tracks into the reference-neutral contract."""
from __future__ import annotations
import hashlib,json,sys
from pathlib import Path
from scipy.io import loadmat

ROOT=Path(__file__).resolve().parents[2]
RAW=ROOT/"validation/datasets/raw/aero4river-v1"
LOCK=ROOT/"validation/datasets/locks/aero4river-v1.sha256"
TRANSFORM=ROOT/"validation/aero4river/aero4river-coordinate-transform-v1.json"
OUT=ROOT/"artifacts/aero4river/imported"

def sha(path:Path)->str:return hashlib.sha256(path.read_bytes()).hexdigest()
def main()->None:
    expected={line.split(maxsplit=1)[1]:line.split(maxsplit=1)[0] for line in LOCK.read_text().splitlines() if line.strip()}
    transform=json.loads(TRANSFORM.read_text())
    if not transform["status"].startswith("resolved-"):raise RuntimeError("AERO4River coordinate transform is unresolved")
    OUT.mkdir(parents=True,exist_ok=True)
    for index in range(1,4):
        name=f"Validation_Experimental_Tests_track{index}.mat";path=RAW/name
        if sha(path)!=expected[name]:raise RuntimeError(f"Checksum mismatch: {name}")
        data=loadmat(path,squeeze_me=True)
        arrays={key:data[key].reshape(-1).tolist() for key in ["x","y","yaw","u","v","r","fx","fy","tn","t"]}
        count=len(arrays["t"])
        if any(len(values)!=count for values in arrays.values()):raise RuntimeError(f"Shape mismatch: {name}")
        samples=[{"time_s":arrays["t"][i],"north_m":arrays["x"][i],"east_m":arrays["y"][i],"down_m":0,"roll_rad":0,"pitch_rad":0,"yaw_rad":arrays["yaw"][i],"u_mps":arrays["u"][i],"v_mps":arrays["v"][i],"w_mps":0,"p_rad_s":0,"q_rad_s":0,"r_rad_s":arrays["r"][i],"input_wrench_body_n_nm":[arrays["fx"][i],arrays["fy"][i],0,0,0,arrays["tn"][i]]} for i in range(count)]
        artifact={"metadata":{"schema_version":1,"reference":"AERO4River","dataset_version":"v1","vessel":"AERO4River","maneuver":f"validation-track-{index}","frame":"NED-SNAME","units":"SI","source_checksum_sha256":expected[name],"notes":["Imported with aero4river-to-bcod-ned-sname-v1.","Source x/y origin retained; source planar channels mapped identically after publication and data-consistency review.","tn is lowercase in the MATLAB file although the landing page writes Tn."]},"samples":samples,"import_provenance":{"source_path":str(path.relative_to(ROOT)),"transform_id":transform["id"],"transform_checksum_sha256":sha(TRANSFORM)}}
        (OUT/f"validation-track-{index}.json").write_text(json.dumps(artifact,indent=2)+"\n")
    print(json.dumps({"output":str(OUT),"tracks":3,"transform":transform["id"]},indent=2))
if __name__=="__main__":main()
