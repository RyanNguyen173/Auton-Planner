/* ======================================================================
   Tank drive motion simulator — core. No DOM: runs in the browser and under `node --test`.
   Internal units follow the spec: meters, radians, seconds, math heading
   (0 = +X, counter-clockwise positive). The UI uses inches and compass
   heading (0 = +Y toward the top wall, clockwise positive, like a V5 IMU).
   ====================================================================== */
const IN=0.0254, D2R=Math.PI/180, G=9.81, HALF=72, LIMIT=90;
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
const sgn=v=>v>0?1:v<0?-1:0;
const wrapPi=a=>{a=(a+Math.PI)%(2*Math.PI);if(a<0)a+=2*Math.PI;return a-Math.PI};
const wrap180=a=>{a=(a+180)%360;if(a<0)a+=360;return a-180};
const norm360=h=>((h%360)+360)%360;
const h2t=h=>(90-h)*D2R, t2h=t=>norm360(90-t/D2R);
const bearing=(fx,fy,tx,ty)=>Math.atan2(tx-fx,ty-fy)/D2R;   /* compass degrees */
const f1=n=>(Math.round(n*10)/10).toFixed(1), f2=n=>(+n).toFixed(2);
let uid=1;const nid=()=>'c'+(uid++)+Math.random().toString(36).slice(2,6);

/* ---------------- Configuration ---------------- */
function defCfg(){return{
  robot:{w:24,l:24},
  drive:{wheel:3.25,cart:600,gIn:36,gOut:48,motors:8,mass:20,eff:80,track:19},
  real:{kS:0.5,kVpct:100,kApct:100,ang:1.4,mu:0.9,trackPct:100,odoScale:0,gyroDrift:0},
  ff:null,
  traj:{v:50,a:100,d:100,lat:140},
  ram:{b:2,zeta:0.7},
  pp:{lmin:6,lmax:18,c:0.6},
  lat:{kP:1.2,kI:0,kD:0.08,settle:1,settleMs:120},
  ang:{kP:0.6,kI:0,kD:0.06,settle:1.5,settleMs:120},
  stop:'brake',
  ex:{hold:1.5,holdD:0.2,coast:30,glideA:60,glideW:360,steer:3,minV:4,minW:30}};}
/* Fill in settings and command fields added in later versions so older saved projects still load */
function normalizeProject(p){const d=defCfg();p.cfg.ex={...d.ex,...(p.cfg.ex||{})};if(!p.cfg.stop)p.cfg.stop=p.cfg.brake===false?'coast':'brake';delete p.cfg.brake;
  p.cmds.forEach(c=>{const def=CMDS[c.type]&&CMDS[c.type].def;if(def)for(const k in def)if(!(k in c))c[k]=JSON.parse(JSON.stringify(def[k]))});return p}

/* Robot "truth": DC-motor style model per drive side. voltage = kS·sgn(v) + kV·v + kA·a */
function plantOf(C){
  const d=C.drive,r=C.real,rpm=d.cart*d.gIn/d.gOut,D=Math.max(0.5,d.wheel)*IN,vFree=rpm/60*Math.PI*D,eff=clamp(d.eff,1,100)/100;
  const stall={100:2.1,200:1.05,600:0.35}[d.cart]||0.35;
  const F=Math.max(1,d.motors)/2*stall*(d.gOut/d.gIn)/(D/2)*eff, mSide=Math.max(1,d.mass)*0.4536/2, aStall=F/mSide;
  const kV0=12/(vFree*eff), kA0=12/aStall;
  return{rpm,vFree,vMax:vFree*eff,aStall,kS:Math.max(0,r.kS),kV:kV0*r.kVpct/100,kA:kA0*r.kApct/100,kV0,kA0,ang:Math.max(0.2,r.ang),
    aTrac:Math.max(0.05,r.mu)*G,coast:clamp((C.ex?C.ex.coast:30)/100,0,1),W:d.track*IN,Weff:d.track*IN*r.trackPct/100,odoScale:r.odoScale/100,gyro:-r.gyroDrift*D2R};
}
function ffFromPlant(P){return{kS:+P.kS.toFixed(3),kV:+P.kV.toFixed(3),kA:+P.kA.toFixed(3),kP:2}}

/* One physics substep. Each side: net = u − kS·sgn(v) − kV·v. The common part accelerates the robot,
   the differential part turns it (scaled by the angular inertia factor). Traction caps wheel acceleration. */
function physStep(S,P,dt){
  /* Coast = motor leads open: no back-EMF braking, only friction and a little drag. Brake = 0 V across a shorted motor (the kV term). */
  const net=S.coast?((u,v)=>Math.abs(v)<1e-4?0:-P.kS*sgn(v)-P.coast*P.kV*v):((u,v)=>{if(Math.abs(v)<1e-4){if(Math.abs(u)<=P.kS)return 0;return u-P.kS*sgn(u)}return u-P.kS*sgn(v)-P.kV*v});
  const nL=net(S.uL,S.vL),nR=net(S.uR,S.vR),c=(nL+nR)/2,dd=(nR-nL)/2;
  let aL=c/P.kA-dd/(P.kA*P.ang),aR=c/P.kA+dd/(P.kA*P.ang);
  const m=Math.max(Math.abs(aL),Math.abs(aR));if(m>P.aTrac){aL*=P.aTrac/m;aR*=P.aTrac/m}
  const vL0=S.vL,vR0=S.vR;S.vL+=aL*dt;S.vR+=aR*dt;
  if(vL0!==0&&sgn(S.vL)!==sgn(vL0)&&(S.coast||Math.abs(S.uL)<=P.kS))S.vL=0;
  if(vR0!==0&&sgn(S.vR)!==sgn(vR0)&&(S.coast||Math.abs(S.uR)<=P.kS))S.vR=0;
  S.pL=(S.pL||0)+S.vL*dt;S.pR=(S.pR||0)+S.vR*dt;
  const v=(S.vL+S.vR)/2,w=(S.vR-S.vL)/P.Weff,tm=S.th+w*dt/2;
  S.x+=v*Math.cos(tm)*dt;S.y+=v*Math.sin(tm)*dt;S.th+=w*dt;
  const vo=v*(1+P.odoScale),wo=w+P.gyro,to=S.oth+wo*dt/2;
  S.ox+=vo*Math.cos(to)*dt;S.oy+=vo*Math.sin(to)*dt;S.oth+=wo*dt;
}

/* ---------------- Bezier path & trajectory (spec stage 1) ---------------- */
function segsOf(c){const A=c.anchors||[],out=[];for(let i=0;i+1<A.length;i++){const a=A[i],b=A[i+1];out.push([{x:a.x,y:a.y},{x:a.x+a.ox,y:a.y+a.oy},{x:b.x+b.ix,y:b.y+b.iy},{x:b.x,y:b.y}])}return out}
const bz=(s,t)=>{const u=1-t;return{x:u*u*u*s[0].x+3*u*u*t*s[1].x+3*u*t*t*s[2].x+t*t*t*s[3].x,y:u*u*u*s[0].y+3*u*u*t*s[1].y+3*u*t*t*s[2].y+t*t*t*s[3].y}};
const bzd=(s,t)=>{const u=1-t;return{x:3*u*u*(s[1].x-s[0].x)+6*u*t*(s[2].x-s[1].x)+3*t*t*(s[3].x-s[2].x),y:3*u*u*(s[1].y-s[0].y)+6*u*t*(s[2].y-s[1].y)+3*t*t*(s[3].y-s[2].y)}};
const bzdd=(s,t)=>({x:6*(1-t)*(s[2].x-2*s[1].x+s[0].x)+6*t*(s[3].x-2*s[2].x+s[1].x),y:6*(1-t)*(s[2].y-2*s[1].y+s[0].y)+6*t*(s[3].y-2*s[2].y+s[1].y)});

function genTraj(c,C){
  const segs=segsOf(c).map(sg=>sg.map(p=>({x:p.x*IN,y:p.y*IN})));if(!segs.length)return null;
  const W=C.drive.track*IN,vmax=Math.max(0.05,C.traj.v*IN*c.spd/100),amax=Math.max(0.05,C.traj.a*IN),dmax=Math.max(0.05,C.traj.d*IN),alat=C.traj.lat*IN;
  const dense=[];let s=0;
  segs.forEach((sg,si)=>{const N=400;for(let k=si?1:0;k<=N;k++){const t=k/N,p=bz(sg,t);if(dense.length){const q=dense[dense.length-1];s+=Math.hypot(p.x-q.x,p.y-q.y)}dense.push({si,t,s,x:p.x,y:p.y})}});
  const total=s;if(total<0.02)return null;
  const n=Math.max(2,Math.ceil(total/0.01)),step=total/n,pts=[];let j=0;
  for(let i=0;i<=n;i++){const target=i*step;while(j<dense.length-2&&dense[j+1].s<target)j++;const a=dense[j],b=dense[j+1];
    const u=clamp((target-a.s)/((b.s-a.s)||1),0,1),si=b.si,ta=a.si===b.si?a.t:0,t=ta+(b.t-ta)*u,sg=segs[si];
    const p=bz(sg,t),d1=bzd(sg,t),d2=bzdd(sg,t),sp=Math.hypot(d1.x,d1.y);
    pts.push({x:p.x,y:p.y,phi:sp>1e-6?Math.atan2(d1.y,d1.x):null,k:sp>1e-6?(d1.x*d2.y-d1.y*d2.x)/(sp*sp*sp):0})}
  for(let i=0;i<pts.length;i++)if(pts[i].phi===null){const a=pts[Math.max(0,i-1)],b=pts[Math.min(pts.length-1,i+1)];pts[i].phi=Math.atan2(b.y-a.y,b.x-a.x)}
  const L=pts.length-1,vcap=pts.map(p=>{let v=vmax/(1+Math.abs(p.k)*W/2);if(alat>0&&Math.abs(p.k)>1e-6)v=Math.min(v,Math.sqrt(alat/Math.abs(p.k)));return v});
  const v=new Array(L+1);v[0]=0;for(let i=1;i<=L;i++)v[i]=Math.min(vcap[i],Math.sqrt(v[i-1]*v[i-1]+2*amax*step));
  v[L]=0;for(let i=L-1;i>=0;i--)v[i]=Math.min(v[i],Math.sqrt(v[i+1]*v[i+1]+2*dmax*step));
  const tt=[0],acc=[];for(let i=0;i<L;i++){const dt=2*step/Math.max(v[i]+v[i+1],1e-6);tt.push(tt[i]+dt);acc.push((v[i+1]-v[i])/dt)}acc.push(0);
  const rev=!!c.rev,sg=rev?-1:1,st=pts.map((p,i)=>({t:tt[i],x:p.x,y:p.y,th:p.phi+(rev?Math.PI:0),v:sg*v[i],w:v[i]*p.k,a:sg*acc[i],alpha:0}));
  for(let i=0;i<L;i++){st[i].alpha=(st[i+1].w-st[i].w)/Math.max(tt[i+1]-tt[i],1e-6)}
  return{pts:st,T:tt[L],len:total,step,k:pts.map(p=>p.k),vprof:v,aprof:acc,rev};
}
function sampleTraj(tr,t){const P=tr.pts;if(t<=0)return P[0];if(t>=tr.T)return{...P[P.length-1],v:0,w:0,a:0,alpha:0};
  let lo=0,hi=P.length-1;while(hi-lo>1){const m=(lo+hi)>>1;if(P[m].t<=t)lo=m;else hi=m}
  const a=P[lo],b=P[hi],u=(t-a.t)/((b.t-a.t)||1),lerp=k=>a[k]+(b[k]-a[k])*u;
  return{t,x:lerp('x'),y:lerp('y'),th:a.th+wrapPi(b.th-a.th)*u,v:lerp('v'),w:lerp('w'),a:a.a,alpha:a.alpha}}

/* ---------------- Controllers ---------------- */
function PID(g){let i=0,prev=null;return{step(e,dt){if(prev!==null&&sgn(e)!==sgn(prev))i=0;i+=e*dt;const d=prev===null?0:(e-prev)/dt;prev=e;return g.kP*e+clamp(g.kI*i,-4,4)+g.kD*d}}}
function Settle(lim,ms){let tin=0;return(ok,dt)=>{tin=ok?tin+dt:0;return tin>=ms/1000-1e-9}}
function mix(lat,ang,maxV){let l=lat+ang,r=lat-ang;const m=Math.max(Math.abs(l),Math.abs(r));if(m>maxV){l*=maxV/m;r*=maxV/m}return[l,r]}
const poseIn=S=>({x:S.ox/IN,y:S.oy/IN,h:90-S.oth/D2R});   /* odometry pose, inches, unwrapped compass heading */
const CLOSE=7.5;

const MK={
  turnHeading(c,C){const pid=PID(C.ang),maxV=12*c.maxSpd/100,minV=12*c.minSpd/100,set=Settle(0,C.ang.settleMs);let tgt;
    return{init(S){const h=poseIn(S).h;let d=wrap180(c.h-h);if(c.dir==='cw'&&d<0)d+=360;if(c.dir==='ccw'&&d>0)d-=360;tgt=h+d},
      step(S,dt){const e=tgt-poseIn(S).h;let o=clamp(pid.step(e,dt),-maxV,maxV);if(minV>0&&Math.abs(o)<minV)o=sgn(e)*minV;
        let done=null;if(c.exit>0&&Math.abs(e)<c.exit)done='Early exit';else if(set(Math.abs(e)<C.ang.settle,dt))done='Settled';
        return{uL:o,uR:-o,done,eHead:e,ePos:NaN}},
      target(S){return{kind:'turn',h:tgt,unwrapped:true}}}},
  turnPoint(c,C){const pid=PID(C.ang),maxV=12*c.maxSpd/100,minV=12*c.minSpd/100,set=Settle(0,C.ang.settleMs);
    return{init(){},step(S,dt){const p=poseIn(S),e=wrap180(bearing(p.x,p.y,c.x,c.y)+(c.back?180:0)-p.h);let o=clamp(pid.step(e,dt),-maxV,maxV);if(minV>0&&Math.abs(o)<minV)o=sgn(e)*minV;
        let done=null;if(c.exit>0&&Math.abs(e)<c.exit)done='Early exit';else if(set(Math.abs(e)<C.ang.settle,dt))done='Settled';
        return{uL:o,uR:-o,done,eHead:e,ePos:NaN,viz:{aim:{x:c.x,y:c.y}}}},
      target(S){const p=poseIn(S);return{kind:'turn',h:bearing(p.x,p.y,c.x,c.y)+(c.back?180:0)}}}},
  moveToPoint(c,C){const lp=PID(C.lat),ap=PID(C.ang),maxV=12*c.maxSpd/100,minV=12*c.minSpd/100,set=Settle(0,C.lat.settleMs);let close=false;
    return{init(){},step(S,dt){const p=poseIn(S),dist=Math.hypot(c.x-p.x,c.y-p.y),rh=p.h+(c.back?180:0),da=wrap180(bearing(p.x,p.y,c.x,c.y)-rh);
        if(!close&&dist<CLOSE)close=true;
        const latE=dist*Math.cos(da*D2R);let lat=clamp(lp.step(latE,dt),-maxV,maxV);const ang=close?0:clamp(ap.step(da,dt),-maxV,maxV);
        if(minV>0&&Math.abs(lat)<minV)lat=sgn(latE||1)*minV;if(c.back)lat=-lat;
        const[l,r]=mix(lat,ang,maxV);let done=null;
        if((c.exit>0&&dist<c.exit)||(minV>0&&latE<0))done='Early exit';else if(set(Math.abs(latE)<C.lat.settle&&(Math.cos(da*D2R)>0.5||dist<2*C.lat.settle),dt))done='Settled';
        return{uL:l,uR:r,done,ePos:dist,eHead:close?NaN:da,viz:{aim:{x:c.x,y:c.y}}}},
      target(){return{kind:'drive',x:c.x,y:c.y,back:c.back}}}},
  moveToPose(c,C){const lp=PID(C.lat),ap=PID(C.ang),maxV=12*c.maxSpd/100,minV=12*c.minSpd/100,set=Settle(0,Math.max(C.lat.settleMs,C.ang.settleMs)),ff=C.ff;let close=false;
    return{init(){},step(S,dt){const p=poseIn(S),dist=Math.hypot(c.x-p.x,c.y-p.y),dir=c.back?-1:1,hr=c.h*D2R;
        if(!close&&dist<CLOSE)close=true;
        const ax=close?c.x:c.x-dir*Math.sin(hr)*c.lead*dist,ay=close?c.y:c.y-dir*Math.cos(hr)*c.lead*dist;
        const rh=p.h+(c.back?180:0),dAim=Math.hypot(ax-p.x,ay-p.y);
        const angE=close?wrap180(c.h-p.h):wrap180(bearing(p.x,p.y,ax,ay)-rh);
        const latE=close?dist*Math.cos(wrap180(bearing(p.x,p.y,c.x,c.y)-rh)*D2R):dAim*Math.cos(angE*D2R);
        let lat=clamp(lp.step(latE,dt),-maxV,maxV);
        if(!close){const sa=Math.abs(Math.sin(angE*D2R));if(sa>1e-3){const rad=dAim/(2*sa)*IN,vs=Math.sqrt(C.traj.lat*IN*rad),vl=ff.kS+ff.kV*vs;lat=clamp(lat,-vl,vl)}}
        const ang=clamp(ap.step(angE,dt),-maxV,maxV);
        if(minV>0&&Math.abs(lat)<minV)lat=sgn(latE||1)*minV;if(c.back)lat=-lat;
        const[l,r]=mix(lat,ang,maxV);let done=null;
        if(c.exit>0&&dist<c.exit)done='Early exit';else if(set(close&&Math.abs(latE)<C.lat.settle&&Math.abs(angE)<C.ang.settle,dt))done='Settled';
        return{uL:l,uR:r,done,ePos:dist,eHead:angE,viz:{aim:{x:c.x,y:c.y},carrot:close?null:{x:ax,y:ay}}}},
      target(){return{kind:'drive',x:c.x,y:c.y,h:c.h,back:c.back}}}},
  followPath(c,C,P,tr){const W=C.drive.track*IN,ff=C.ff,tol=c.tol*IN,lp=PID(C.lat),set=Settle(0,C.lat.settleMs);let t0,idx=0,settle=false;
    const ffv=(v,a,meas)=>clamp(ff.kS*(Math.abs(v)>1e-3?sgn(v):0)+ff.kV*v+ff.kA*a+ff.kP*(v-meas),-12,12);
    const n=tr.pts.length,pts=tr.pts,end=pts[n-1];
    return{init(S){t0=S.t},step(S,dt){const el=S.t-t0,px=S.ox,py=S.oy,pth=S.oth;
      if(c.ctrl==='ramsete'){
        /* Spec stage 2: RAMSETE on the time-parameterized trajectory, stage 3: feedforward + velocity P */
        const d=sampleTraj(tr,el),dx=d.x-px,dy=d.y-py,ex=Math.cos(pth)*dx+Math.sin(pth)*dy,ey=-Math.sin(pth)*dx+Math.cos(pth)*dy,et=wrapPi(d.th-pth);
        const b=C.ram.b,z=C.ram.zeta,k=2*z*Math.sqrt(d.w*d.w+b*d.v*d.v),sinc=Math.abs(et)<1e-6?1:Math.sin(et)/et;
        const v=d.v*Math.cos(et)+k*ex,w=d.w+k*et+b*d.v*sinc*ey,vL=v-w*W/2,vR=v+w*W/2,aL=d.a-d.alpha*W/2,aR=d.a+d.alpha*W/2;
        const endErr=Math.hypot(end.x-px,end.y-py);let done=null;if(c.exit>0&&el>tr.T*0.5&&endErr<c.exit*IN)done='Early exit';else if(el>=tr.T&&endErr<tol)done='Settled';
        return{uL:ffv(vL,aL,S.vL),uR:ffv(vR,aR,S.vR),done,tL:vL/IN,tR:vR/IN,ePos:Math.hypot(dx,dy)/IN,eHead:-et/D2R,viz:{des:{x:d.x/IN,y:d.y/IN,h:t2h(d.th)}}}}
      /* Pure pursuit (spec fallback) */
      let best=idx,bd=Infinity;for(let j=idx;j<Math.min(n,idx+150);j++){const q=pts[j],dd=(q.x-px)**2+(q.y-py)**2;if(dd<bd){bd=dd;best=j}}idx=best;
      const th=pth+(c.rev?Math.PI:0);
      if(c.exit>0&&(n-1-idx)*tr.step<c.exit*IN&&Math.hypot(end.x-px,end.y-py)<(c.exit+2)*IN)return{uL:S.uL,uR:S.uR,done:'Early exit',ePos:Math.sqrt(bd)/IN,eHead:NaN};
      if(!settle&&idx>=n-3)settle=true;
      if(settle){const dx=end.x-px,dy=end.y-py,along=(Math.cos(th)*dx+Math.sin(th)*dy)/IN;let o=clamp(lp.step(along,dt),-6,6);if(c.rev)o=-o;
        const endErr=Math.hypot(dx,dy)/IN;let done=null;if(set(Math.abs(along)<Math.max(c.tol,C.lat.settle),dt))done='Settled';
        return{uL:o,uR:o,done,ePos:endErr,eHead:-wrapPi(pts[n-1].th-pth)/D2R,viz:{look:{x:end.x/IN,y:end.y/IN},L:0}}}
      const kk=Math.abs(tr.k[idx]),Lm=clamp(kk>1e-6?C.pp.c/kk:1e9,C.pp.lmin*IN,C.pp.lmax*IN);
      let li=-1;for(let j=idx;j<n;j++){const q=pts[j];if(Math.hypot(q.x-px,q.y-py)>=Lm){li=j;break}}
      /* Near the end, extend the path along its final tangent so the lookahead distance never collapses */
      let q;if(li>=0)q=pts[li];else{const ph=end.th-(c.rev?Math.PI:0),ux=Math.cos(ph),uy=Math.sin(ph),ex=end.x-px,ey=end.y-py,along=ex*ux+ey*uy,perp2=ex*ex+ey*ey-along*along,ext=Math.sqrt(Math.max(0,Lm*Lm-perp2))-along;q={x:end.x+ux*Math.max(0,ext),y:end.y+uy*Math.max(0,ext)}}
      const dx=q.x-px,dy=q.y-py,ly=-Math.sin(th)*dx+Math.cos(th)*dy,L2=dx*dx+dy*dy,gam=L2>1e-6?2*ly/L2:0;
      const vm=Math.max(tr.vprof[Math.min(n-1,idx+1)],0.05),a=tr.aprof[idx];
      const vLv=vm*(1-gam*W/2),vRv=vm*(1+gam*W/2);
      const[vL,vR,aL,aR]=c.rev?[-vRv,-vLv,-a,-a]:[vLv,vRv,a,a];
      return{uL:ffv(vL,aL,S.vL),uR:ffv(vR,aR,S.vR),done:null,tL:vL/IN,tR:vR/IN,ePos:Math.sqrt(bd)/IN,eHead:-wrapPi(pts[idx].th-pth)/D2R,viz:{look:{x:q.x/IN,y:q.y/IN},L:Math.sqrt(L2)/IN}}},
      target(){return{kind:'drive',x:end.x/IN,y:end.y/IN,h:t2h(end.th),back:!!c.rev}}}},
  wait(c,C){let t0,hold;return{init(S){t0=S.t;hold={L:S.pL||0,R:S.pR||0}},step(S){return{...stopOut(C.stop,S,C,hold),done:S.t-t0>=c.ms/1000-1e-9?'Done':null,ePos:NaN,eHead:NaN}}}},
};

/* ---------------- Stopping and early-exit behavior ----------------
   coast: motor leads open, the robot rolls on friction alone
   brake: 0 V across a shorted motor, back-EMF slows it hard
   hold:  position hold on each side (V5 "hold")
   guide: keep driving toward the target with a glide speed profile v = √(2·a·distance), never faster than the
          robot is already going (unless below the minimum guide speed), while steering at it */
function stopOut(mode,S,C,hold){
  if(mode==='coast')return{uL:0,uR:0,coast:true};
  if(mode==='hold'){const k=C.ex.hold,d=C.ex.holdD;return{uL:clamp(-k*((S.pL||0)-hold.L)/IN-d*S.vL/IN,-12,12),uR:clamp(-k*((S.pR||0)-hold.R)/IN-d*S.vR/IN,-12,12)}}
  return{uL:0,uR:0}}
function guideCtrl(tg,C){const W=C.drive.track*IN,ff=C.ff,ex=C.ex;
  const ffv=(v,meas)=>clamp(ff.kS*(Math.abs(v)>1e-3?sgn(v):0)+ff.kV*v+ff.kP*(v-meas),-12,12);
  return{step(S){const p=poseIn(S),v=(S.vL+S.vR)/2,wc=(S.vL-S.vR)/W;   /* wc: compass turn rate, clockwise positive */
    if(tg.kind==='turn'){const e=tg.unwrapped?tg.h-p.h:wrap180(tg.h-p.h),glide=Math.sqrt(2*ex.glideW*Math.abs(e)),cur=Math.abs(wc)/D2R;
      let wt=Math.abs(e)<0.3?0:sgn(e)*Math.min(glide,Math.max(cur,ex.minW))*D2R;const vL=wt*W/2,vR=-wt*W/2;
      return{uL:ffv(vL,S.vL),uR:ffv(vR,S.vR),tL:vL/IN,tR:vR/IN,eHead:e,ePos:NaN,done:Math.abs(e)<0.3&&Math.abs(wc)<0.05?'Arrived':null}}
    const dist=Math.hypot(tg.x-p.x,tg.y-p.y),rh=p.h+(tg.back?180:0),da=wrap180(bearing(p.x,p.y,tg.x,tg.y)-rh),along=dist*Math.cos(da*D2R),vin=Math.abs(v)/IN;
    /* Past the target: creep back at no more than the minimum guide speed */
    const vt=along>0.25?Math.min(Math.sqrt(2*ex.glideA*along),Math.max(vin,ex.minV)):along<-0.25?-Math.min(Math.sqrt(2*ex.glideA*-along),ex.minV):0;
    const steerDeg=dist>3?da:(tg.h!=null?wrap180(tg.h-p.h):0),wt=clamp(ex.steer*steerDeg*D2R,-4,4),vl=(tg.back?-1:1)*vt*IN;
    const vL=vl+wt*W/2,vR=vl-wt*W/2;
    return{uL:ffv(vL,S.vL),uR:ffv(vR,S.vR),tL:vL/IN,tR:vR/IN,ePos:dist,eHead:steerDeg,done:Math.abs(along)<=0.25&&vin<0.5?'Arrived':null,viz:{aim:{x:tg.x,y:tg.y}}}}}}

/* ---------------- Sequence runner: 10 ms control loop, 2 ms physics ---------------- */
function simulate(proj){
  const C=proj.cfg,P=plantOf(C),dt=0.01,sub=5;
  const S={t:0,x:proj.start.x*IN,y:proj.start.y*IN,th:h2t(proj.start.h),vL:0,vR:0,uL:0,uR:0,pL:0,pR:0,coast:false};S.ox=S.x;S.oy=S.y;S.oth=S.th;
  const log=[],res=[],trajs={};
  const snap=(ci,o)=>log.push({t:S.t,x:S.x/IN,y:S.y/IN,h:t2h(S.th),ox:S.ox/IN,oy:S.oy/IN,oh:t2h(S.oth),vL:S.vL/IN,vR:S.vR/IN,uL:S.uL,uR:S.uR,
    tL:o&&o.tL!=null?o.tL:NaN,tR:o&&o.tR!=null?o.tR:NaN,ci,ph:o&&o.ph||null,ePos:o?o.ePos:NaN,eHead:o?o.eHead:NaN,viz:o&&o.viz||null});
  snap(-1,null);let stopped=false;
  proj.cmds.forEach((c,ci)=>{
    if(stopped){res.push(null);return}
    const startPose={x:S.ox/IN,y:S.oy/IN,h:t2h(S.oth)},trueStart={x:S.x/IN,y:S.y/IN,h:t2h(S.th)};
    let tr=null;if(c.type==='followPath'){tr=genTraj(c,C);trajs[c.id]=tr;if(!tr){res.push({t0:S.t,t1:S.t,reason:'Empty path',sat:0,startPose,trueStart});return}}
    const ctrl=MK[c.type](c,C,P,tr);ctrl.init(S);
    const t0=S.t,timeout=c.type==='wait'?1e9:c.type==='followPath'?(c.timeout>0?c.timeout/1000:tr.T+1.5):c.timeout/1000;
    let sat=0,ticks=0,reason=null;
    for(;;){const o=ctrl.step(S,dt);S.uL=clamp(o.uL,-12,12);S.uR=clamp(o.uR,-12,12);S.coast=!!o.coast;if(Math.abs(S.uL)>11.9||Math.abs(S.uR)>11.9)sat++;ticks++;
      if(o.done)reason=o.done;else if(S.t-t0>=timeout-1e-9)reason='Timeout';
      snap(ci,o);if(reason)break;
      for(let k=0;k<sub;k++)physStep(S,P,dt/sub);S.t=Math.round((S.t+dt)*1000)/1000;
      if(S.t>=LIMIT){reason='Time limit';stopped=true;break}}
    let tExit=null,exitPose=null;
    if(reason==='Early exit'&&c.exitMode&&c.exitMode!=='chain'&&!stopped){
      tExit=S.t;exitPose={x:S.x/IN,y:S.y/IN,h:t2h(S.th)};const tg=ctrl.target?ctrl.target(S):null,hold={L:S.pL,R:S.pR},g=c.exitMode==='guide'&&tg?guideCtrl(tg,C):null,lim=S.t+Math.max(0,c.exitMs)/1000;
      for(;;){const o=g?g.step(S,dt):{...stopOut(c.exitMode,S,C,hold),ePos:NaN,eHead:NaN};o.ph='exit';
        S.uL=clamp(o.uL,-12,12);S.uR=clamp(o.uR,-12,12);S.coast=!!o.coast;snap(ci,o);
        if(S.t>=lim-1e-9||(g&&o.done)||(!g&&Math.abs(S.vL)+Math.abs(S.vR)<0.01&&S.t-tExit>0.05))break;
        for(let k=0;k<sub;k++)physStep(S,P,dt/sub);S.t=Math.round((S.t+dt)*1000)/1000;if(S.t>=LIMIT){stopped=true;break}}
      S.coast=false}
    const tp={x:S.x/IN,y:S.y/IN,h:t2h(S.th)};let posErr=NaN,headErr=NaN,startOff=NaN;
    if(c.type==='moveToPoint'){posErr=Math.hypot(c.x-tp.x,c.y-tp.y)}
    else if(c.type==='moveToPose'){posErr=Math.hypot(c.x-tp.x,c.y-tp.y);headErr=wrap180(c.h-tp.h)}
    else if(c.type==='turnHeading'){headErr=wrap180(c.h-tp.h)}
    else if(c.type==='turnPoint'){headErr=wrap180(bearing(tp.x,tp.y,c.x,c.y)+(c.back?180:0)-tp.h)}
    else if(c.type==='followPath'){const e=tr.pts[tr.pts.length-1];posErr=Math.hypot(e.x/IN-tp.x,e.y/IN-tp.y);headErr=wrap180(t2h(e.th)-tp.h);const s0=tr.pts[0];startOff=Math.hypot(s0.x/IN-trueStart.x,s0.y/IN-trueStart.y)}
    res.push({t0,t1:S.t,reason,sat:ticks?sat/ticks:0,startPose,trueStart,endPose:tp,posErr,headErr,startOff,tExit,exitPose,exitMode:tExit!=null?c.exitMode:null});
  });
  const holdEnd={L:S.pL,R:S.pR};for(let k=0;k<40;k++){const o=stopOut(C.stop,S,C,holdEnd);S.uL=o.uL;S.uR=o.uR;S.coast=!!o.coast;for(let j=0;j<sub;j++)physStep(S,P,dt/sub);S.t=Math.round((S.t+dt)*1000)/1000;snap(-2,null)}
  return{log,res,trajs,P,T:S.t};
}
function logAt(L,t){if(t<=L[0].t)return 0;if(t>=L[L.length-1].t)return L.length-1;let lo=0,hi=L.length-1;while(hi-lo>1){const m=(lo+hi)>>1;if(L[m].t<=t)lo=m;else hi=m}return lo}

/* ---------------- Commands & example routine ---------------- */
const CMDS={
  moveToPoint:{name:'Move to point',def:{x:0,y:0,back:false,maxSpd:100,minSpd:0,timeout:2000,exit:0,exitMode:'chain',exitMs:400}},
  moveToPose:{name:'Move to pose',def:{x:0,y:0,h:0,lead:0.6,back:false,maxSpd:100,minSpd:0,timeout:2500,exit:0,exitMode:'chain',exitMs:400}},
  turnHeading:{name:'Turn to heading',def:{h:90,dir:'auto',maxSpd:100,minSpd:0,timeout:1500,exit:0,exitMode:'chain',exitMs:400}},
  turnPoint:{name:'Turn to point',def:{x:0,y:0,back:false,maxSpd:100,minSpd:0,timeout:1500,exit:0,exitMode:'chain',exitMs:400}},
  followPath:{name:'Follow Bezier path',def:{ctrl:'ramsete',rev:false,spd:100,tol:1.5,timeout:0,exit:0,exitMode:'chain',exitMs:400,anchors:[]}},
  wait:{name:'Wait',def:{ms:300}},
};
const A=(x,y,ix,iy,ox,oy)=>({x,y,ix,iy,ox,oy});
function exampleProject(){const cfg=defCfg();cfg.ff=ffFromPlant(plantOf(cfg));
  const mk=(type,o)=>({id:nid(),type,...JSON.parse(JSON.stringify(CMDS[type].def)),...o});
  return{v:1,start:{x:-60,y:-40,h:90},cfg,cmds:[
    mk('followPath',{ctrl:'ramsete',anchors:[A(-60,-40,0,0,16,0),A(-26,-18,-11,-11,11,11),A(-12,12,0,-14,0,14)]}),
    mk('turnHeading',{h:270}),
    mk('moveToPoint',{x:-48,y:12,exit:6,exitMode:'guide',exitMs:600}),
    mk('turnPoint',{x:-24,y:-48,exit:10,exitMode:'brake',exitMs:300}),
    mk('moveToPose',{x:-24,y:-48,h:180,lead:0.5}),
    mk('wait',{ms:250}),
    mk('followPath',{ctrl:'purepursuit',rev:true,anchors:[A(-24,-48,0,0,0,16),A(-48,-12,0,-20,0,20)]}),
  ]};
}

export{IN,D2R,G,HALF,LIMIT,clamp,sgn,wrapPi,wrap180,norm360,h2t,t2h,bearing,f1,f2,nid,defCfg,normalizeProject,plantOf,ffFromPlant,physStep,
  segsOf,bz,bzd,bzdd,genTraj,sampleTraj,PID,Settle,mix,poseIn,MK,stopOut,guideCtrl,simulate,logAt,CMDS,exampleProject};
