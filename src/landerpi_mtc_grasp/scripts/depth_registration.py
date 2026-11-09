"""Project metric depth into the calibrated RGB optical frame, with z-buffering."""
import numpy as np

def register(depth,depth_k,rgb_k,rotation,translation,height,width):
    if depth.ndim!=2 or not np.isfinite(depth_k).all() or not np.isfinite(rgb_k).all() or min(depth_k[0],depth_k[4],rgb_k[0],rgb_k[4])<=0:
        raise ValueError('invalid depth image or camera intrinsics')
    v,u=np.nonzero(np.isfinite(depth)&(depth>0))
    z=depth[v,u]
    points=np.column_stack(((u-depth_k[2])*z/depth_k[0],(v-depth_k[5])*z/depth_k[4],z))
    points=points@np.asarray(rotation).T+np.asarray(translation)
    points=points[np.isfinite(points).all(axis=1)&(points[:,2]>.01)]
    u=np.rint(rgb_k[0]*points[:,0]/points[:,2]+rgb_k[2]).astype(int)
    v=np.rint(rgb_k[4]*points[:,1]/points[:,2]+rgb_k[5]).astype(int)
    valid=(u>=0)&(u<width)&(v>=0)&(v<height)
    output=np.full(height*width,np.inf,dtype=np.float32)
    np.minimum.at(output,v[valid]*width+u[valid],points[valid,2])
    output[~np.isfinite(output)]=0
    return output.reshape(height,width)
