#pragma OPENCL FP_CONTRACT OFF
typedef struct { float4 r0; float4 r1; float4 r2; uint4 meta; } Pose;
typedef uchar MapBucket;
inline int occupancy(int3 c,__global const MapBucket* grid,uint4 info,int3 origin) {
 int3 relative=c-origin;
 if(any(relative<0)||relative.x>=(int)info.y||relative.y>=(int)info.z||relative.z>=(int)info.w)return 0;
 uint index=((uint)relative.z*info.z+(uint)relative.y)*info.y+(uint)relative.x;
 return grid[info.x+index]!=0;
}
inline int2 evaluate(Pose pose,float4 p,__global const MapBucket* buckets,uint4 info,int3 origin,float inv) {
 float3 q=(float3)(
  (((pose.r0.x*p.x+pose.r0.y*p.y)+pose.r0.z*p.z)+pose.r0.w)*inv,
  (((pose.r1.x*p.x+pose.r1.y*p.y)+pose.r1.z*p.z)+pose.r1.w)*inv,
  (((pose.r2.x*p.x+pose.r2.y*p.y)+pose.r2.z*p.z)+pose.r2.w)*inv);
 float3 mag=(float3)(
  ((fabs(pose.r0.x*p.x)+fabs(pose.r0.y*p.y))+fabs(pose.r0.z*p.z))+fabs(pose.r0.w),
  ((fabs(pose.r1.x*p.x)+fabs(pose.r1.y*p.y))+fabs(pose.r1.z*p.z))+fabs(pose.r1.w),
  ((fabs(pose.r2.x*p.x)+fabs(pose.r2.y*p.y))+fabs(pose.r2.z*p.z))+fabs(pose.r2.w));
 float3 error=mag*(16.0f*0x1.0p-23f*inv)+(float3)(1.0e-6f);
 int ambiguous=any(convert_int3_rtn(q-error)!=convert_int3_rtn(q+error));
 return (int2)(occupancy(convert_int3_rtn(q),buckets,info,origin),ambiguous);
}
__kernel void score_serial(__global const Pose* poses,uint nc,
 __global const float4* points,uint np,__global const MapBucket* buckets,
 __global const uint4* mapinfo,__global const float* inverses,
 __global int4* output,__global uint* masks,uint words) {
 uint c=get_global_id(0);if(c>=nc)return;
 Pose pose=poses[c];uint4 info=mapinfo[pose.meta.x*2u];int3 origin=as_int4(mapinfo[pose.meta.x*2u+1u]).xyz;float inv=inverses[pose.meta.x];
 int score=0,ambiguous_hits=0,ambiguous_points=0;uint bits=0;
 for(uint p=0;p<np;++p) {
  int2 v=evaluate(pose,points[p],buckets,info,origin,inv);
  score+=v.x;
  if(v.y){bits|=1u<<(p%32u);ambiguous_hits+=v.x;ambiguous_points++;}
  if(p%32u==31u || p+1==np){masks[c*words+p/32u]=bits;bits=0;}
 }
 output[c]=(int4)(score,ambiguous_hits,ambiguous_points,0);
}
__kernel void score_group(__global const Pose* poses,uint nc,
 __global const float4* points,uint np,__global const MapBucket* buckets,
 __global const uint4* mapinfo,__global const float* inverses,
 __global int4* output,__global uint* masks,uint words) {
 uint c=get_group_id(0),lid=get_local_id(0);if(c>=nc)return;
 Pose pose=poses[c];uint4 info=mapinfo[pose.meta.x*2u];int3 origin=as_int4(mapinfo[pose.meta.x*2u+1u]).xyz;float inv=inverses[pose.meta.x];
 int score=0,ah=0,ap=0;
 for(uint p=lid;p<np;p+=64u) {
  int2 v=evaluate(pose,points[p],buckets,info,origin,inv);score+=v.x;
  if(v.y){atomic_or(&masks[c*words+p/32u],1u<<(p%32u));ah+=v.x;ap++;}
 }
 __local int scores[64],hits[64],counts[64];
 scores[lid]=score;hits[lid]=ah;counts[lid]=ap;barrier(CLK_LOCAL_MEM_FENCE);
 for(uint step=32;step>0;step>>=1){if(lid<step){scores[lid]+=scores[lid+step];hits[lid]+=hits[lid+step];counts[lid]+=counts[lid+step];}barrier(CLK_LOCAL_MEM_FENCE);}
 if(lid==0)output[c]=(int4)(scores[0],hits[0],counts[0],0);
}

