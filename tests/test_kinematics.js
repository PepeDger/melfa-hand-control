'use strict';
const assert=require('node:assert/strict');
const {fk,mv}=require('../kinematics.js');
const near=(a,b,eps=1e-8)=>assert.ok(Math.abs(a-b)<eps,`${a} != ${b}`);
// Manufacturer's reference posture: J2=0, J3=90, forearm horizontal.
const horizontal=fk([0,0,90,0,0,0]);
near(horizontal.pts[2][2],595);
near(horizontal.pts[4][2],595);
near(horizontal.p[0],355);
// Measured installation: J2+J3-90+J5=90.02°, flange points down.
const actual=[1.53,-8.26,157.49,-.01,30.79,91.52];
const before=actual.slice();
const actualPose=fk(actual);
const direction=mv(actualPose.R,[1,0,0]);
assert.ok(direction[2]<-.99999,'Measured pose should point the tool down');
assert.ok(actualPose.pts[4][2]<actualPose.pts[2][2],'Forearm should descend from the elbow');
assert.deepEqual(actual,before,'Rendering must never modify controller joint angles');
// Wrist roll changes orientation, keeping flange position fixed.
const rollA=fk([0,0,90,0,0,0]),rollB=fk([0,0,90,0,0,90]);
rollA.p.forEach((v,i)=>near(v,rollB.p[i]));
assert.notDeepEqual(rollA.R,rollB.R);
assert.throws(()=>fk([NaN,0,0,0,0,0]));
console.log('Kinematics: reference posture, measured pose, unchanged input and wrist roll OK');
