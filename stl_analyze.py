import struct, collections

with open(r"E:\workfold\framework\build\JEAMMSCAN.stl","rb") as f:
    f.read(80)
    n = struct.unpack("<I", f.read(4))[0]
    tris=[]
    for _ in range(n):
        f.read(12)  # normal
        p0=struct.unpack("<3f", f.read(12))
        p1=struct.unpack("<3f", f.read(12))
        p2=struct.unpack("<3f", f.read(12))
        f.read(2)   # attr
        tris.append((p0,p1,p2))

print("tris:", n)
# weld vertices by rounding
def key(p): return tuple(round(c,4) for c in p)
vk={}
verts=[]
for t in tris:
    for p in t:
        k=key(p)
        if k not in vk:
            vk[k]=len(verts); verts.append(k)

idx=[]
for t in tris:
    idx.append([vk[key(p)] for p in t])

# edge count
edge_count=collections.Counter()
for (a,b,c) in idx:
    for (u,v) in ((a,b),(b,c),(c,a)):
        if u<v: edge_count[(u,v)]+=1
        else:   edge_count[(v,u)]+=1

boundary=sum(1 for e,c in edge_count.items() if c!=2)
print("unique verts:", len(verts))
print("unique edges:", len(edge_count))
print("boundary edges (used !=2 times):", boundary)

# winding consistency: for each directed edge, check how often it appears with matching orientation
orient=collections.Counter()
for (a,b,c) in idx:
    orient[(a,b)]+=1; orient[(b,c)]+=1; orient[(c,a)]+=1
    orient[(b,a)]-=1; orient[(c,b)]-=1; orient[(a,c)]-=1
bad=sum(1 for (e,c) in orient.items() if c!=0)
print("inconsistently-wound directed edges:", bad)

# normal orientation vs winding: cross product direction
import numpy as np
def cross(p0,p1,p2):
    return np.cross(np.array(p1)-np.array(p0), np.array(p2)-np.array(p0))
file_norm_matches=0
for t,(a,b,c) in zip(tris,idx):
    cr=cross(t[0],t[1],t[2])
    cr/= (np.linalg.norm(cr)+1e-12)
    # compare to a fixed outward-ish heuristic: normal points away from centroid
print("done")
