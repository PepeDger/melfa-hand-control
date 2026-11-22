/* Display only: never use this approximation to generate robot commands.
 * Reference: Mitsubishi BFP-A8653-N, 2-29/2-30 (J3=90 at horizontal forearm).
 * Dimensions: BFP-A8652-D fig. 2-3, nominal arm 245/270 mm, flange 85 mm.
 * Link bodies, transverse offsets and installed tool are not a calibrated CAD/DH model.
 */
(function(root){
 'use strict';
 const add=(a,b)=>a.map((v,i)=>v+b[i]);
 const mul=(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((s,v,k)=>s+v*b[k][j],0)));
 const mv=(a,v)=>a.map(r=>r.reduce((s,x,i)=>s+x*v[i],0));
 function rot(axis,deg){
  const a=deg*Math.PI/180,c=Math.cos(a),s=Math.sin(a);
  return axis==='z'?[[c,-s,0],[s,c,0],[0,0,1]]:
         axis==='y'?[[c,0,s],[0,1,0],[-s,0,c]]:[[1,0,0],[0,c,-s],[0,s,c]];
 }
 function fk(q){
  if(q.length!==6||!q.every(Number.isFinite))throw Error('Six finite joint angles required');
  let R=rot('z',q[0]);
  const shoulder=[0,0,350];
  const frames=[[[0,0,175],R],[shoulder,R]];
  R=mul(R,rot('y',q[1]));
  const elbow=add(shoulder,mv(R,[0,0,245]));
  frames.push([elbow,R]);
  R=mul(R,rot('y',q[2]-90));
  R=mul(R,rot('x',q[3]));
  const twist=add(elbow,mv(R,[40,0,0]));
  frames.push([twist,R]);
  const wrist=add(elbow,mv(R,[270,0,0]));
  frames.push([wrist,R]);
  R=mul(R,rot('y',q[4]));
  const flange=add(wrist,mv(R,[85,0,0]));
  R=mul(R,rot('x',q[5]));
  frames.push([flange,R]);
  return {pts:[[0,0,0],shoulder,elbow,twist,wrist,flange],frames,R,p:flange};
 }
 const api={add,mul,mv,rot,fk};
 if(typeof module!=='undefined'&&module.exports)module.exports=api;
 else root.MelfaKinematics=api;
})(typeof globalThis!=='undefined'?globalThis:this);
