/* Motion simulator view: build a command routine (move to point/pose, turns, Bezier paths with RAMSETE or
   pure pursuit, waits), run it on the modeled tank drive from core.js, and inspect telemetry.
   Scoped to the view's root element. DOM ids in this view are prefixed "s-" so they never collide with the planner. */
import {IN,D2R,HALF,clamp,sgn,wrap180,norm360,t2h,bearing,f1,f2,nid,defCfg,normalizeProject,plantOf,ffFromPlant,segsOf,bz,simulate,logAt,CMDS,exampleProject} from './core.js';
import {confirmDelete,isModalOpen,registerTips,onThemeChange,downloadBlob,esc} from '../shared/ui.js';
import {GOALS,TOGGLES,TOGGLE_SIZE,LOADERS,OBJECTS,LZ} from '../shared/field-data.js';

/**
 * @param {HTMLElement} root  the [data-pane="sim"] section
 * @param {{isActive:()=>boolean}} hooks
 */
export function initSim(root,hooks){
  const $=s=>root.querySelector(s);
  const SAVEKEY='tank-sim-project-v1';
  let proj=loadProject()||exampleProject();
  let R=null;
  const state={sel:0,selAnchor:null,time:0,playing:false,rate:1,tab:'routine',chart:'speed',snap:3,showOdom:true,showAll:true,
    acc:{size:true,real:false,ff:false,traj:true,ram:false,pp:false,lat:false,ang:false,misc:false}};

  function loadProject(){try{const p=JSON.parse(localStorage.getItem(SAVEKEY)||'null');return validProject(p)?normalizeProject(p):null}catch(_){return null}}
  function validProject(p){return p&&p.start&&p.cfg&&Array.isArray(p.cmds)&&p.cfg.drive&&p.cfg.real&&p.cfg.traj&&p.cfg.lat&&p.cfg.ang}
  let saveT=0;function save(){clearTimeout(saveT);saveT=setTimeout(()=>{try{localStorage.setItem(SAVEKEY,JSON.stringify(proj))}catch(_){}},400)}

  function syncFF(){if(proj.cfg.ffAuto!==false||!proj.cfg.ff){const kP=proj.cfg.ff?proj.cfg.ff.kP:2;proj.cfg.ff={...ffFromPlant(plantOf(proj.cfg)),kP}}}
  function resim(){syncFF();R=simulate(proj);state.time=clamp(state.time,0,R.T);save()}
  const selCmd=()=>proj.cmds[state.sel]||null;

  /* ---------------- Colors & canvas ---------------- */
  let CV={};const VARS=['--paper','--surface','--surface-sunken','--ink','--ink-muted','--ink-subtle','--line','--line-strong','--accent','--on-accent','--accent-ink','--accent-wash','--inverse','--on-inverse','--success','--warning','--danger','--red','--blue','--yellow','--field','--tileA','--tileB','--seam','--tape','--sideL','--sideR'];
  const readVars=()=>{const cs=getComputedStyle(document.documentElement);CV={};VARS.forEach(k=>CV[k]=cs.getPropertyValue(k).trim())};
  const alpha=(c,a)=>{const m=/^#([0-9a-f]{6})$/i.exec(c||'');if(!m)return c;const n=parseInt(m[1],16);return`rgba(${n>>16},${(n>>8)&255},${n&255},${a})`};
  const RAMP=[[91,63,209],[25,179,166],[255,210,63]];
  const speedCol=u=>{u=clamp(u,0,1);const s=u<.5?0:1,k=u<.5?u*2:(u-.5)*2,a=RAMP[s],b=RAMP[s+1];return`rgb(${a.map((v,i)=>Math.round(v+(b[i]-v)*k)).join(',')})`};
  let T={ox:0,oy:0,s:1};
  const px=x=>T.ox+(x+HALF)*T.s,py=y=>T.oy+(HALF-y)*T.s,wx=X=>(X-T.ox)/T.s-HALF,wy=Y=>HALF-(Y-T.oy)/T.s;
  function fit(cv){const r=cv.getBoundingClientRect(),d=window.devicePixelRatio||1,w=Math.max(1,Math.round(r.width*d)),h=Math.max(1,Math.round(r.height*d));if(cv.width!==w||cv.height!==h){cv.width=w;cv.height=h}const c=cv.getContext('2d');c.setTransform(d,0,0,d,0,0);return{c,w:r.width,h:r.height}}
  const FONT='Public Sans, system-ui, sans-serif',MONO='Geist Mono, ui-monospace, monospace';

  /* ---------------- Override field (VEX GPS coordinates, manual v2.1) ---------------- */
  const allyCol=a=>a==='red'?CV['--red']:a==='blue'?CV['--blue']:CV['--yellow'];
  function drawField(c){
    const s=T.s,x0=px(-HALF),y0=py(HALF),W=144*s;
    for(let i=0;i<6;i++)for(let j=0;j<6;j++){c.fillStyle=(i+j)%2?CV['--tileA']:CV['--tileB'];c.fillRect(x0+i*24*s,y0+j*24*s,24*s+0.5,24*s+0.5)}
    c.strokeStyle=CV['--seam'];c.lineWidth=1;for(let i=1;i<6;i++){c.beginPath();c.moveTo(x0+i*24*s,y0);c.lineTo(x0+i*24*s,y0+W);c.moveTo(x0,y0+i*24*s);c.lineTo(x0+W,y0+i*24*s);c.stroke()}
    [['red',[[-HALF,HALF],[-12,12],[-24,0],[-12,-12],[-HALF,-HALF]]],['red',[[-HALF,-HALF],[-12,-12],[0,-24],[12,-12],[HALF,-HALF]]],['blue',[[HALF,-HALF],[12,-12],[24,0],[12,12],[HALF,HALF]]],['blue',[[HALF,HALF],[12,12],[0,24],[-12,12],[-HALF,HALF]]]].forEach(([a,pts])=>{c.fillStyle=alpha(allyCol(a),0.07);c.beginPath();pts.forEach(([x,y],i)=>i?c.lineTo(px(x),py(y)):c.moveTo(px(x),py(y)));c.closePath();c.fill()});
    c.save();c.lineWidth=Math.max(1.2,s*0.5);[[-1,1,'red'],[-1,-1,'red'],[1,1,'blue'],[1,-1,'blue']].forEach(([sx,sy,a])=>{c.strokeStyle=allyCol(a);const ix=sx*(HALF-LZ.w),iy=sy*(HALF-LZ.h);c.beginPath();c.moveTo(px(ix),py(sy*HALF));c.lineTo(px(ix),py(iy));c.lineTo(px(sx*HALF),py(iy));c.stroke()});c.restore();
    c.save();c.strokeStyle=CV['--tape'];c.lineWidth=Math.max(1.2,s*0.5);const seg=(a,b)=>{c.beginPath();c.moveTo(px(a[0]),py(a[1]));c.lineTo(px(b[0]),py(b[1]));c.stroke()};
    seg([-HALF,-HALF],[-12,-12]);seg([HALF,HALF],[12,12]);const o=1.5/Math.SQRT2;[[-o,-o],[o,o]].forEach(([dx,dy])=>{seg([-HALF+dx+0.8,HALF+dy-0.8],[-12+dx,12+dy]);seg([12+dx,-12+dy],[HALF+dx-0.8,-HALF+dy+0.8])});
    c.beginPath();[[24,0],[0,24],[-24,0],[0,-24]].forEach(([x,y],i)=>i?c.lineTo(px(x),py(y)):c.moveTo(px(x),py(y)));c.closePath();c.stroke();c.restore();
    c.lineWidth=2;c.strokeStyle=CV['--ink'];c.strokeRect(x0,y0,W,W);
    c.fillStyle=CV['--red'];c.fillRect(x0-6,y0,3,W);c.fillStyle=CV['--blue'];c.fillRect(x0+W+3,y0,3,W);
    LOADERS.forEach(l=>{c.fillStyle=alpha(allyCol(l.a),0.8);c.fillRect(px(l.x-3),py(l.y+3.5),6*s,7*s)});
    TOGGLES.forEach(t=>{c.fillStyle=CV['--yellow'];const lw=TOGGLE_SIZE.len,th=TOGGLE_SIZE.face;if(t.horiz)c.fillRect(px(t.x-lw/2),py(t.y+th/2),lw*s,th*s);else c.fillRect(px(t.x-th/2),py(t.y+lw/2),th*s,lw*s)});
    OBJECTS.forEach(o=>{c.fillStyle=alpha(CV['--yellow'],0.5);c.beginPath();c.arc(px(o.x),py(o.y),Math.max(2,1.4*s),0,7);c.fill()});
    GOALS.forEach(gl=>{c.fillStyle=gl.type==='neutral'?CV['--surface-sunken']:alpha(allyCol(gl.type),0.85);c.strokeStyle=CV['--ink'];c.lineWidth=gl.tall?2:1.2;c.beginPath();c.arc(px(gl.x),py(gl.y),(gl.tall?5:3.5)*s,0,7);c.fill();c.stroke()});
  }

  /* ---------------- Scene ---------------- */
  const fieldCv=$('#s-field'),chartCv=$('#s-chart');
  function drawRobot(c,p,opts={}){const s=T.s,w=proj.cfg.robot.w*s,l=proj.cfg.robot.l*s;c.save();c.translate(px(p.x),py(p.y));c.rotate(p.h*D2R);
    if(opts.ghost){c.setLineDash(opts.dash||[5,4]);c.strokeStyle=opts.color||CV['--ink-muted'];c.lineWidth=1.5;c.strokeRect(-w/2,-l/2,w,l);c.setLineDash([]);c.beginPath();c.moveTo(0,-l/2);c.lineTo(0,-l/2-8);c.stroke();c.restore();return}
    c.fillStyle=alpha(CV['--accent'],0.9);c.strokeStyle=CV['--ink'];c.lineWidth=2;c.beginPath();c.roundRect?c.roundRect(-w/2,-l/2,w,l,Math.min(5,w*.1)):c.rect(-w/2,-l/2,w,l);c.fill();c.stroke();
    c.fillStyle=CV['--ink'];c.beginPath();c.moveTo(0,-l/2+3);c.lineTo(Math.min(9,w*.22),-l/2+Math.min(14,l*.32));c.lineTo(-Math.min(9,w*.22),-l/2+Math.min(14,l*.32));c.closePath();c.fill();
    c.fillStyle=alpha(CV['--ink'],0.55);c.fillRect(-w/2-2,-l/3,3,l*2/3);c.fillRect(w/2-1,-l/3,3,l*2/3);c.restore()}
  function badge(c,x,y,n,sel){c.save();c.fillStyle=sel?CV['--inverse']:CV['--accent'];c.strokeStyle=CV['--paper'];c.lineWidth=2;c.beginPath();c.arc(x,y,9,0,7);c.fill();c.stroke();c.fillStyle=sel?CV['--on-inverse']:CV['--on-accent'];c.font=`500 11px ${MONO}`;c.textAlign='center';c.textBaseline='middle';c.fillText(String(n),x,y+.5);c.restore()}
  function arrow(c,x,y,hdg,len,col,lw=2){const a=hdg*D2R,ex=x+Math.sin(a)*len,ey=y-Math.cos(a)*len;c.save();c.strokeStyle=col;c.fillStyle=col;c.lineWidth=lw;c.beginPath();c.moveTo(x,y);c.lineTo(ex,ey);c.stroke();c.beginPath();c.moveTo(ex,ey);c.lineTo(ex-Math.sin(a-0.45)*8,ey+Math.cos(a-0.45)*8);c.lineTo(ex-Math.sin(a+0.45)*8,ey+Math.cos(a+0.45)*8);c.closePath();c.fill();c.restore()}
  const startOf=i=>R&&R.res[i]?R.res[i].startPose:i>0&&R&&R.res[i-1]?R.res[i-1].endPose:proj.start;
  const knobOf=(c)=>{const a=c.h*D2R;return{x:c.x+Math.sin(a)*14,y:c.y+Math.cos(a)*14}};
  const startKnob=()=>{const a=proj.start.h*D2R,r=proj.cfg.robot.l/2+6;return{x:proj.start.x+Math.sin(a)*r,y:proj.start.y+Math.cos(a)*r}};
  function drawCmd(c,cmd,i,sel){
    const n=i+1,ink=CV['--accent-ink'],dim=sel?1:0.55;c.save();c.globalAlpha=dim;
    if(cmd.type==='followPath'){const segs=segsOf(cmd);if(segs.length){c.strokeStyle=ink;c.lineWidth=sel?3:2;c.setLineDash(cmd.rev?[8,6]:[]);c.beginPath();c.moveTo(px(segs[0][0].x),py(segs[0][0].y));segs.forEach(s=>c.bezierCurveTo(px(s[1].x),py(s[1].y),px(s[2].x),py(s[2].y),px(s[3].x),py(s[3].y)));c.stroke();c.setLineDash([]);
        const m=bz(segs[Math.floor(segs.length/2)],segs.length%2?0.5:0);badge(c,px(m.x),py(m.y),n,sel)}
      if(sel){cmd.anchors.forEach((a,k)=>{c.strokeStyle=alpha(CV['--ink'],0.6);c.lineWidth=1;c.beginPath();if(k>0){c.moveTo(px(a.x),py(a.y));c.lineTo(px(a.x+a.ix),py(a.y+a.iy))}if(k<cmd.anchors.length-1){c.moveTo(px(a.x),py(a.y));c.lineTo(px(a.x+a.ox),py(a.y+a.oy))}c.stroke();
        [[k>0,a.ix,a.iy],[k<cmd.anchors.length-1,a.ox,a.oy]].forEach(([on,dx,dy])=>{if(!on)return;c.fillStyle=CV['--surface'];c.strokeStyle=CV['--ink'];c.lineWidth=1.5;c.fillRect(px(a.x+dx)-4,py(a.y+dy)-4,8,8);c.strokeRect(px(a.x+dx)-4,py(a.y+dy)-4,8,8)});
        c.fillStyle=state.selAnchor===k?CV['--accent']:CV['--surface'];c.strokeStyle=CV['--ink'];c.lineWidth=2;c.beginPath();c.arc(px(a.x),py(a.y),6,0,7);c.fill();c.stroke()})}}
    else if(cmd.type==='turnHeading'){const sp=startOf(i);arrow(c,px(sp.x),py(sp.y),cmd.h,22+T.s*6,ink,sel?3:2);const a=cmd.h*D2R;badge(c,px(sp.x)+Math.sin(a)*(30+T.s*6),py(sp.y)-Math.cos(a)*(30+T.s*6),n,sel)}
    else if(cmd.type==='turnPoint'){const sp=startOf(i);c.strokeStyle=ink;c.setLineDash([3,4]);c.lineWidth=1.5;c.beginPath();c.moveTo(px(sp.x),py(sp.y));c.lineTo(px(cmd.x),py(cmd.y));c.stroke();c.setLineDash([]);
      c.lineWidth=2;c.beginPath();c.arc(px(cmd.x),py(cmd.y),11,0,7);c.stroke();c.beginPath();c.moveTo(px(cmd.x)-15,py(cmd.y));c.lineTo(px(cmd.x)+15,py(cmd.y));c.moveTo(px(cmd.x),py(cmd.y)-15);c.lineTo(px(cmd.x),py(cmd.y)+15);c.stroke();badge(c,px(cmd.x)+13,py(cmd.y)-13,n,sel)}
    else if(cmd.type==='moveToPoint'||cmd.type==='moveToPose'){
      if(cmd.type==='moveToPose'){arrow(c,px(cmd.x),py(cmd.y),cmd.h,14*T.s,ink,sel?3:2);if(sel){const k=knobOf(cmd);c.fillStyle=CV['--accent'];c.strokeStyle=CV['--ink'];c.lineWidth=1.5;c.beginPath();c.arc(px(k.x),py(k.y),6,0,7);c.fill();c.stroke()}}
      badge(c,px(cmd.x),py(cmd.y),n,sel)}
    c.restore()}
  function drawScene(){
    readVars();const{c,w,h}=fit(fieldCv),pad=18,S=Math.max(0.05,(Math.min(w,h)-pad*2)/144);T={ox:(w-144*S)/2,oy:(h-144*S)/2,s:S};c.clearRect(0,0,w,h);
    drawField(c);if(!R)return;
    const L=R.log,i=logAt(L,state.time),cur=L[i],vmax=Math.max(R.P.vMax/IN,1);
    proj.cmds.forEach((cmd,ci)=>{if(state.showAll||ci===state.sel)if(ci!==state.sel)drawCmd(c,cmd,ci,false)});
    c.save();c.lineCap='round';c.strokeStyle=alpha(CV['--ink'],0.18);c.lineWidth=2;c.beginPath();L.forEach((q,k)=>k?c.lineTo(px(q.x),py(q.y)):c.moveTo(px(q.x),py(q.y)));c.stroke();
    c.lineWidth=4;for(let k=1;k<=i;k++){const a=L[k-1],b=L[k];c.strokeStyle=speedCol(Math.abs(b.vL+b.vR)/2/vmax);c.beginPath();c.moveTo(px(a.x),py(a.y));c.lineTo(px(b.x),py(b.y));c.stroke()}c.restore();
    if(selCmd())drawCmd(c,selCmd(),state.sel,true);
    R.res.forEach((r,ri)=>{if(!r||!r.exitPose||!(state.showAll||ri===state.sel))return;const x=px(r.exitPose.x),y=py(r.exitPose.y);c.save();c.strokeStyle=CV['--warning'];c.fillStyle=CV['--paper'];c.lineWidth=2;c.beginPath();c.moveTo(x,y-7);c.lineTo(x+7,y);c.lineTo(x,y+7);c.lineTo(x-7,y);c.closePath();c.fill();c.stroke();c.restore()});
    drawRobot(c,{...proj.start},{ghost:true,dash:[2,4],color:CV['--ink-subtle']});const sk=startKnob();c.save();c.fillStyle=CV['--accent'];c.strokeStyle=CV['--ink'];c.lineWidth=1.5;c.beginPath();c.arc(px(sk.x),py(sk.y),5.5,0,7);c.fill();c.stroke();c.restore();
    const v=cur.viz;
    if(v){if(v.des)drawRobot(c,v.des,{ghost:true,color:CV['--accent-ink']});
      if(v.carrot){c.save();c.strokeStyle=CV['--warning'];c.fillStyle=CV['--warning'];c.setLineDash([4,4]);c.lineWidth=1.5;c.beginPath();c.moveTo(px(cur.ox),py(cur.oy));c.lineTo(px(v.carrot.x),py(v.carrot.y));c.stroke();c.setLineDash([]);c.beginPath();c.moveTo(px(v.carrot.x),py(v.carrot.y)-7);c.lineTo(px(v.carrot.x)+7,py(v.carrot.y));c.lineTo(px(v.carrot.x),py(v.carrot.y)+7);c.lineTo(px(v.carrot.x)-7,py(v.carrot.y));c.closePath();c.fill();c.restore()}
      if(v.look){c.save();c.strokeStyle=CV['--success'];c.fillStyle=CV['--success'];c.lineWidth=1.5;if(v.L>0){c.setLineDash([3,4]);c.beginPath();c.arc(px(cur.ox),py(cur.oy),v.L*T.s,0,7);c.stroke();c.setLineDash([])}c.beginPath();c.moveTo(px(cur.ox),py(cur.oy));c.lineTo(px(v.look.x),py(v.look.y));c.stroke();c.beginPath();c.arc(px(v.look.x),py(v.look.y),5,0,7);c.fill();c.restore()}
      if(v.aim&&!v.carrot&&cur.ci>=0&&proj.cmds[cur.ci]&&proj.cmds[cur.ci].type!=='moveToPose'){c.save();c.strokeStyle=alpha(CV['--accent-ink'],0.7);c.setLineDash([4,4]);c.lineWidth=1.5;c.beginPath();c.moveTo(px(cur.ox),py(cur.oy));c.lineTo(px(v.aim.x),py(v.aim.y));c.stroke();c.restore()}}
    if(state.showOdom&&Math.hypot(cur.ox-cur.x,cur.oy-cur.y)>0.3)drawRobot(c,{x:cur.ox,y:cur.oy,h:cur.oh},{ghost:true,dash:[1,3],color:CV['--warning']});
    drawRobot(c,cur);
  }

  /* ---------------- Telemetry chart ---------------- */
  const CH={pl:46,pr:46,pt:22,pb:22};
  function drawChart(){
    const{c,w,h}=fit(chartCv);c.clearRect(0,0,w,h);if(!R)return;
    const L=R.log,Tm=Math.max(R.T,0.1),X=t=>CH.pl+t/Tm*(w-CH.pl-CH.pr),y0=CH.pt,y1=h-CH.pb;
    R.res.forEach((r,i)=>{if(!r)return;const a=X(r.t0),b=X(r.t1);if(i%2===0){c.fillStyle=alpha(CV['--accent'],0.08);c.fillRect(a,y0,b-a,y1-y0)}
      if(b-a>14){c.fillStyle=i===state.sel?CV['--ink']:CV['--ink-muted'];c.font=`400 11px ${MONO}`;c.textAlign='center';c.fillText(String(i+1),(a+b)/2,12)}
      if(i===state.sel){c.fillStyle=CV['--accent-ink'];c.fillRect(a,y0-3,Math.max(2,b-a),2)}
      if(r.tExit!=null){const e=X(r.tExit);c.save();c.beginPath();c.rect(e,y0,b-e,y1-y0);c.clip();c.strokeStyle=alpha(CV['--warning'],0.35);c.lineWidth=1;for(let k=e-(y1-y0);k<b;k+=6){c.beginPath();c.moveTo(k,y1);c.lineTo(k+(y1-y0),y0);c.stroke()}c.restore();
        c.strokeStyle=CV['--warning'];c.setLineDash([3,3]);c.beginPath();c.moveTo(e,y0);c.lineTo(e,y1);c.stroke();c.setLineDash([])}});
    c.font=`400 11px ${MONO}`;c.lineWidth=1;
    const grid=(lo,hi,Y,step,fmt,side)=>{c.textAlign=side==='r'?'left':'right';for(let v=Math.ceil(lo/step)*step;v<=hi+1e-9;v+=step){const yy=Y(v);c.strokeStyle=Math.abs(v)<1e-9?CV['--line-strong']:CV['--line'];if(side!=='r'){c.beginPath();c.moveTo(CH.pl,yy);c.lineTo(w-CH.pr,yy);c.stroke()}c.fillStyle=CV['--ink-muted'];c.fillText(fmt(v),side==='r'?w-CH.pr+6:CH.pl-6,yy+4)}};
    const series=(key,Y,col,dash,lw=1.75)=>{c.strokeStyle=col;c.lineWidth=lw;c.setLineDash(dash||[]);c.beginPath();let on=false;L.forEach(q=>{const v=q[key];if(!isFinite(v)){on=false;return}const xx=X(q.t),yy=Y(v);on?c.lineTo(xx,yy):c.moveTo(xx,yy);on=true});c.stroke();c.setLineDash([])};
    const nice=m=>{const s=[1,2,5,10,20,25,50,100,200];const raw=m/4;return s.find(v=>v>=raw)||200};
    const i=logAt(L,state.time),q=L[i];let cap='';
    if(state.chart==='speed'){let m=R.P.vMax/IN;L.forEach(p=>{for(const k of ['vL','vR','tL','tR'])if(isFinite(p[k]))m=Math.max(m,Math.abs(p[k]))});m*=1.08;const Y=v=>y0+(m-v)/(2*m)*(y1-y0);
      grid(-m,m,Y,nice(m),v=>v,'l');c.setLineDash([2,4]);c.strokeStyle=CV['--ink-subtle'];[R.P.vMax/IN,-R.P.vMax/IN].forEach(v=>{c.beginPath();c.moveTo(CH.pl,Y(v));c.lineTo(w-CH.pr,Y(v));c.stroke()});c.setLineDash([]);
      series('tL',Y,alpha(CV['--sideL'],0.85),[5,4],1.5);series('tR',Y,alpha(CV['--sideR'],0.85),[5,4],1.5);series('vL',Y,CV['--sideL']);series('vR',Y,CV['--sideR']);
      cap=`Left <b style="color:var(--sideL)">${f1(q.vL)}</b> and right <b style="color:var(--sideR)">${f1(q.vR)}</b> in/s actual${isFinite(q.tL)?` · target ${f1(q.tL)} / ${f1(q.tR)} (dashed)`:''}. Dotted lines mark the robot's top speed.`}
    else if(state.chart==='volt'){const m=13,Y=v=>y0+(m-v)/(2*m)*(y1-y0);grid(-12,12,Y,4,v=>v,'l');c.setLineDash([4,3]);c.strokeStyle=CV['--danger'];[12,-12].forEach(v=>{c.beginPath();c.moveTo(CH.pl,Y(v));c.lineTo(w-CH.pr,Y(v));c.stroke()});c.setLineDash([]);
      series('uL',Y,CV['--sideL']);series('uR',Y,CV['--sideR']);
      cap=`Left <b style="color:var(--sideL)">${f2(q.uL)} V</b> · right <b style="color:var(--sideR)">${f2(q.uR)} V</b>. Time spent on the red 12 V lines is saturated: the controller asked for more than the motors can give.`}
    else{let mp=1,mh=2;L.forEach(p=>{if(isFinite(p.ePos))mp=Math.max(mp,Math.abs(p.ePos));if(isFinite(p.eHead))mh=Math.max(mh,Math.abs(p.eHead))});mp=Math.min(mp*1.1,200);mh=Math.min(mh*1.1,200);
      const Yp=v=>y0+(mp-v)/(2*mp)*(y1-y0),Yh=v=>y0+(mh-v)/(2*mh)*(y1-y0);grid(-mp,mp,Yp,nice(mp),v=>v,'l');grid(-mh,mh,Yh,nice(mh),v=>v+'°','r');
      series('ePos',Yp,CV['--accent-ink']);series('eHead',Yh,CV['--warning']);
      cap=`<b style="color:var(--accent-ink)">Position error</b> ${isFinite(q.ePos)?f2(q.ePos)+' in':'—'} (left axis) · <b style="color:var(--warning)">heading error</b> ${isFinite(q.eHead)?f1(q.eHead)+'°':'—'} (right axis). Errors are what the controller sees from odometry.`}
    c.fillStyle=CV['--ink-muted'];c.textAlign='center';const st=Tm>20?2:Tm>8?1:0.5;for(let t=0;t<=Tm+1e-9;t+=st)c.fillText((st<1?t.toFixed(1):t)+'s',X(t),h-5);
    c.strokeStyle=CV['--ink'];c.lineWidth=1.5;c.beginPath();c.moveTo(X(state.time),y0-4);c.lineTo(X(state.time),y1);c.stroke();
    $('#s-chartCap').innerHTML=cap;
  }

  /* ---------------- Transport ---------------- */
  const ICON={play:'<svg class="i" viewBox="0 0 16 16" aria-hidden="true"><path d="M5 3.5v9l7-4.5z"/></svg>',pause:'<svg class="i" viewBox="0 0 16 16" aria-hidden="true"><path d="M5.5 3.5v9M10.5 3.5v9"/></svg>',
    x:'<svg class="i" viewBox="0 0 16 16" aria-hidden="true"><path d="M4 4l8 8M12 4l-8 8"/></svg>',up:'<svg class="i" viewBox="0 0 16 16" aria-hidden="true"><path d="M8 12.5v-9M4.5 7L8 3.5 11.5 7"/></svg>',down:'<svg class="i" viewBox="0 0 16 16" aria-hidden="true"><path d="M8 3.5v9M4.5 9L8 12.5 11.5 9"/></svg>',
    copy:'<svg class="i" viewBox="0 0 16 16" aria-hidden="true"><rect x="5.5" y="5.5" width="8" height="8" rx="1.5"/><path d="M10.5 3.5v-1h-8v8h1"/></svg>',check:'<svg class="i" viewBox="0 0 12 12" aria-hidden="true"><path d="M2.5 6.5l2.2 2.2L9.5 3.5"/></svg>',alert:'<svg class="i" viewBox="0 0 12 12" aria-hidden="true"><path d="M6 3v3.5M6 9h.01"/></svg>',
    chev:'<svg class="v-acc-chevron" width="20" height="20" viewBox="0 0 20 20" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M5 7.5l5 5 5-5"/></svg>'};
  const reasonTag=r=>!r?'':r.reason==='Settled'?`<span class="pill good">${ICON.check}Settled</span>`:r.reason==='Timeout'||r.reason==='Time limit'?`<span class="pill bad">${ICON.alert}Timeout</span>`:r.reason==='Early exit'?`<span class="pill accent">Early exit</span>`:r.reason==='Empty path'?`<span class="pill warn">${ICON.alert}Empty</span>`:`<span class="pill">Done</span>`;
  function updateTransport(){
    $('#s-clock').innerHTML=`${f2(state.time)} <small>/ ${f2(R.T)} s</small>`;
    const pb=$('#s-playBtn'),want=state.playing?'pause':'play';if(pb.dataset.s!==want){pb.dataset.s=want;pb.innerHTML=state.playing?ICON.pause+'Pause':ICON.play+'Run';pb.setAttribute('aria-label',state.playing?'Pause':'Run')}
    const rs=R.res.filter(Boolean),to=rs.filter(r=>r.reason==='Timeout'||r.reason==='Time limit').length,worst=Math.max(0,...rs.map(r=>r.sat));
    const end=rs.length?rs[rs.length-1].t1:0;
    $('#s-summary').innerHTML=`<span class="pill accent">Routine ${f2(end)} s</span>${proj.source&&proj.source.planned?`<span class="pill">Planner estimate ${f2(proj.source.planned)} s</span>`:''}${to?`<span class="pill bad">${ICON.alert}${to} timeout${to>1?'s':''}</span>`:`<span class="pill good">${ICON.check}All settled</span>`}<span class="pill ${worst>0.5?'warn':''}">${worst>0.5?ICON.alert:''}Peak saturation ${Math.round(worst*100)}%</span>`;
  }
  function drawAll(){if(root.hidden)return;drawScene();drawChart();updateTransport()}

  /* ---------------- Panel ---------------- */
  const FIELDS={
    moveToPoint:[['x','X','pos'],['y','Y','pos'],['back','Drive backwards','bool'],['maxSpd','Max speed','pct'],['minSpd','Min speed','pct'],['timeout','Timeout','ms'],['exit','Early exit range','in','x'],['exitMode','After early exit','exitmode','x'],['exitMs','Exit phase limit','ms','x']],
    moveToPose:[['x','X','pos'],['y','Y','pos'],['h','End heading','deg'],['lead','Lead','lead'],['back','Drive backwards','bool'],['maxSpd','Max speed','pct'],['minSpd','Min speed','pct'],['timeout','Timeout','ms'],['exit','Early exit range','in','x'],['exitMode','After early exit','exitmode','x'],['exitMs','Exit phase limit','ms','x']],
    turnHeading:[['h','Heading','deg'],['dir','Direction','dir'],['maxSpd','Max speed','pct'],['minSpd','Min speed','pct'],['timeout','Timeout','ms'],['exit','Early exit range','degx','x'],['exitMode','After early exit','exitmode','x'],['exitMs','Exit phase limit','ms','x']],
    turnPoint:[['x','X','pos'],['y','Y','pos'],['back','Face it with the back','bool'],['maxSpd','Max speed','pct'],['minSpd','Min speed','pct'],['timeout','Timeout','ms'],['exit','Early exit range','degx','x'],['exitMode','After early exit','exitmode','x'],['exitMs','Exit phase limit','ms','x']],
    followPath:[['ctrl','Controller','ctrl'],['rev','Drive backwards','bool'],['spd','Speed limit','pct'],['tol','End tolerance','in'],['timeout','Timeout (0 = auto)','ms'],['exit','Early exit range','in','x'],['exitMode','After early exit','exitmode','x'],['exitMs','Exit phase limit','ms','x']],
    wait:[['ms','Duration','ms']],
  };
  const EXITMODES=[['chain','Chain into next'],['coast','Coast'],['brake','Brake'],['guide','Guide to target']],EXITNAME=Object.fromEntries(EXITMODES);
  const KIND={pos:['in',0.5],deg:['°',5],degx:['°',0.5],pct:['%',5,'min="0" max="100"'],in:['in',0.5,'min="0"'],ms:['ms',50,'min="0"'],lead:['',0.05,'min="0" max="1"']};
  let fid=0;
  const numRow=(label,bind,val,unit,step,extra='')=>{const id='f'+(fid++);return`<label for="${id}">${label}</label><span><input type="number" id="${id}" data-bind="${bind}" value="${val}" step="${step}" ${extra}><span class="u">${unit}</span></span>`};
  const boolRow=(label,bind,val)=>{const id='f'+(fid++);return`<label for="${id}">${label}</label><span><input type="checkbox" id="${id}" data-bind="${bind}" ${val?'checked':''}></span>`};
  const selRow=(label,bind,val,opts)=>{const id='f'+(fid++);return`<label for="${id}">${label}</label><select id="${id}" data-bind="${bind}">${opts.map(([v,t])=>`<option value="${v}" ${String(val)===String(v)?'selected':''}>${t}</option>`).join('')}</select>`};
  const r2=v=>Math.round(v*100)/100;
  function fieldRow(cmd,[key,label,kind]){const v=cmd[key],b='cmd.'+key;
    if(kind==='bool')return boolRow(label,b,v);
    if(kind==='dir')return selRow(label,b,v,[['auto','Shortest'],['cw','Clockwise'],['ccw','Counter-clockwise']]);
    if(kind==='ctrl')return selRow(label,b,v,[['ramsete','RAMSETE'],['purepursuit','Pure pursuit']]);
    if(kind==='exitmode')return selRow(label,b,v,EXITMODES);
    const[u,st,ex]=KIND[kind];return numRow(label,b,r2(v),u,st,ex||'')}
  function summary(cmd){const ex=cmd.exit>0?` · exit ${cmd.exit}${cmd.type.startsWith('turn')?'°':' in'} ${(EXITNAME[cmd.exitMode]||'').toLowerCase()}`:'';return summary0(cmd)+ex}
  function summary0(cmd){const p=(x,y)=>`(${f1(x)}, ${f1(y)})`;switch(cmd.type){
    case'moveToPoint':return`${p(cmd.x,cmd.y)}${cmd.back?' · backwards':''}`;
    case'moveToPose':return`${p(cmd.x,cmd.y)} · ${f1(norm360(cmd.h))}°${cmd.back?' · backwards':''}`;
    case'turnHeading':return`${f1(norm360(cmd.h))}°${cmd.dir!=='auto'?' · '+(cmd.dir==='cw'?'clockwise':'counter-clockwise'):''}`;
    case'turnPoint':return`${p(cmd.x,cmd.y)}${cmd.back?' · back':''}`;
    case'followPath':return`${cmd.ctrl==='ramsete'?'RAMSETE':'Pure pursuit'} · ${Math.max(0,cmd.anchors.length-1)} segment${cmd.anchors.length===2?'':'s'}${cmd.rev?' · backwards':''}`;
    case'wait':return`${cmd.ms} ms${cmd.note?' · '+cmd.note:''}`}}
  const acc=(id,title,meta,body)=>{const open=!!state.acc[id];return`<div class="v-acc-item" data-open="${open}"><button class="v-acc-trigger" data-acc="${id}" aria-expanded="${open}" aria-controls="acc-${id}"><span>${title}</span><small>${meta}</small>${ICON.chev}</button><div class="v-acc-panel" id="acc-${id}" role="region"><div class="v-acc-inner"><div class="v-acc-body">${body}</div></div></div></div>`};
  function renderPanel(){
    root.querySelectorAll('#s-tabs button').forEach(b=>b.setAttribute('aria-selected',b.dataset.tab===state.tab));
    const C=proj.cfg,P=R.P;let h='';fid=0;
    if(state.tab==='routine'){
      if(proj.source&&proj.source.from==='planner')h+=`<div class="readout">Converted from <b>${esc(proj.source.name)}</b> in the route planner${proj.source.planned?`, which estimated <b>${f2(proj.source.planned)} s</b>`:''}.${proj.source.skipped?` ${proj.source.skipped} “while moving” action${proj.source.skipped>1?'s were':' was'} left out because the simulator has no equivalent.`:''}</div>`;
      h+=`<div><h4>Start pose</h4><div class="rows">${numRow('X','start.x',r2(proj.start.x),'in',0.5)}${numRow('Y','start.y',r2(proj.start.y),'in',0.5)}${numRow('Heading','start.h',r2(proj.start.h),'°',5)}</div>
        <p class="hint" style="margin-top:var(--space-2)">Drag the dotted robot to move the start, or its knob to turn it. Heading 0° faces the top wall and increases clockwise, like a V5 inertial sensor.</p></div>`;
      h+=`<div><h4>Commands</h4><div class="list">${proj.cmds.map((cmd,i)=>{const r=R.res[i];return`<button class="li cmd" data-act="sel" data-i="${i}" aria-current="${i===state.sel}"><span class="n">${i+1}</span><span class="cmdtxt"><span>${CMDS[cmd.type].name}</span><small>${esc(summary(cmd))}</small></span><span class="cmdres"><span class="m">${r?f2(r.t1-r.t0)+' s':'—'}</span>${reasonTag(r)}</span></button>`}).join('')||'<p class="hint" style="padding:var(--space-3) 0">No commands yet. Add one below.</p>'}</div>
        <div class="flexrow" style="margin-top:var(--space-3)"><select id="s-newType" aria-label="Command to add">${Object.entries(CMDS).map(([k,v])=>`<option value="${k}">${v.name}</option>`).join('')}</select><button class="btn" data-act="add"><svg class="i" viewBox="0 0 16 16" aria-hidden="true"><path d="M8 3.5v9M3.5 8h9"/></svg>Add after selected</button></div></div>`;
      const cmd=selCmd();
      if(cmd){const r=R.res[state.sel];
        h+=`<div class="editor"><div class="edhead"><h4>${state.sel+1} · ${CMDS[cmd.type].name}</h4><span class="flexrow" style="gap:2px">
          <button class="btn ghost icon" data-act="up" aria-label="Move up" ${state.sel===0?'disabled':''}>${ICON.up}</button><button class="btn ghost icon" data-act="down" aria-label="Move down" ${state.sel>=proj.cmds.length-1?'disabled':''}>${ICON.down}</button>
          <button class="btn ghost icon" data-act="dup" aria-label="Duplicate">${ICON.copy}</button><button class="btn ghost icon danger" data-act="del" aria-label="Delete command">${ICON.x}</button></span></div>
          <div class="rows">${FIELDS[cmd.type].filter(f=>f[3]!=='x').map(f=>fieldRow(cmd,f)).join('')}</div>`;
        const exf=FIELDS[cmd.type].filter(f=>f[3]==='x');
        if(exf.length)h+=`<div class="exgrp"><h4>Early exit ${cmd.exit>0?`<span class="pill accent">${EXITNAME[cmd.exitMode]||'Chain into next'}</span>`:'<span class="pill">Off</span>'}</h4><div class="rows">${exf.filter(f=>cmd.exit>0||f[0]==='exit').map(f=>fieldRow(cmd,f)).join('')}</div>
          <p class="hint">${cmd.exit>0?(cmd.exitMode==='chain'?'The next command starts the moment the robot is in range, so it keeps its momentum.':cmd.exitMode==='guide'?'After exiting, the robot keeps steering at the target and slows along the glide profile in Tuning.':`After exiting, the drive switches to ${cmd.exitMode} until it stops or the limit runs out.`):'Set a range above 0 to end this command before it fully settles.'}</p></div>`;
        if(cmd.type==='followPath'){const tr=R.trajs[cmd.id];
          h+=`<div><h4 style="margin-top:var(--space-4)">Path points</h4><div class="list">${cmd.anchors.map((a,k)=>`<div class="li anch" aria-current="${state.selAnchor===k}"><span class="n">${k}</span><span class="flexrow" style="gap:6px"><input type="number" aria-label="Point ${k} X" data-bind="anc.${k}.x" value="${r2(a.x)}" step="0.5"><input type="number" aria-label="Point ${k} Y" data-bind="anc.${k}.y" value="${r2(a.y)}" step="0.5"><span class="u">in</span></span><button class="btn ghost icon danger" data-act="delAnc" data-k="${k}" aria-label="Remove point ${k}">${ICON.x}</button></div>`).join('')||''}</div>
            <div class="flexrow" style="margin-top:var(--space-3)"><button class="btn" data-act="snapStart">Start path at robot</button></div>
            ${tr?`<div class="readout" style="margin-top:var(--space-3)">Length <b>${f1(tr.len/IN)} in</b> · profiled time <b>${f2(tr.T)} s</b><br>Peak speed <b>${f1(Math.max(...tr.vprof)/IN)} in/s</b> · tightest radius <b>${f1(1/Math.max(1e-6,Math.max(...tr.k.map(Math.abs)))/IN)} in</b></div>`:''}</div>`}
        if(r){const off=isFinite(r.startOff)&&r.startOff>2;
          h+=`<div class="readout" style="margin-top:var(--space-3)">${reasonTag(r)}<br>Ran <b>${f2(r.t0)}–${f2(r.t1)} s</b> (${f2(r.t1-r.t0)} s)${r.tExit!=null?`<br>Exited early at <b>${f2(r.tExit)} s</b>, then ${(EXITNAME[r.exitMode]||'').toLowerCase()} for <b>${f2(r.t1-r.tExit)} s</b>`:''}<br>Start <b>(${f1(r.trueStart.x)}, ${f1(r.trueStart.y)}) ${f1(r.trueStart.h)}°</b><br>End <b>(${f1(r.endPose.x)}, ${f1(r.endPose.y)}) ${f1(r.endPose.h)}°</b>${isFinite(r.posErr)?`<br>Final position error <b>${f2(r.posErr)} in</b>`:''}${isFinite(r.headErr)?`<br>Final heading error <b>${f1(r.headErr)}°</b>`:''}<br>Saturated <b>${Math.round(r.sat*100)}%</b> of the time${off?`<br><span style="color:var(--warning)">The robot starts ${f1(r.startOff)} in from the path start. Use Start path at robot.</span>`:''}</div>`}
        h+=`</div>`}
    }
    else if(state.tab==='robot'){const d=C.drive,rl=C.real,ff=C.ff;
      h+=`<div class="v-accordion">
      ${acc('size','Footprint and drivetrain',`${Math.round(P.rpm)} rpm · ${f1(P.vMax/IN)} in/s`,`<div class="rows">
        ${selRow('Robot size','cfg.robot.size',[15,18,24].includes(C.robot.w)&&C.robot.w===C.robot.l?C.robot.w:'c',[[15,'15 × 15 in'],[18,'18 × 18 in'],[24,'24 × 24 in'],['c','Custom']])}
        ${numRow('Width','cfg.robot.w',C.robot.w,'in',0.5,'min="4" max="30"')}${numRow('Length','cfg.robot.l',C.robot.l,'in',0.5,'min="4" max="30"')}
        ${numRow('Wheel diameter','cfg.drive.wheel',d.wheel,'in',0.25,'min="1"')}
        ${selRow('Motor cartridge','cfg.drive.cart',d.cart,[[100,'100 rpm (red)'],[200,'200 rpm (green)'],[600,'600 rpm (blue)']])}
        ${numRow('Gear: motor teeth','cfg.drive.gIn',d.gIn,'T',1,'min="1"')}${numRow('Gear: wheel teeth','cfg.drive.gOut',d.gOut,'T',1,'min="1"')}
        ${numRow('Drive motors (total)','cfg.drive.motors',d.motors,'',2,'min="2" max="16"')}${numRow('Robot mass','cfg.drive.mass',d.mass,'lb',0.5,'min="1"')}
        ${numRow('Efficiency','cfg.drive.eff',d.eff,'%',1,'min="30" max="100"')}${numRow('Track width','cfg.drive.track',d.track,'in',0.25,'min="4"')}</div>
        <div class="readout">Free speed <b>${f1(P.vFree/IN)} in/s</b> · real top <b>${f1(P.vMax/IN)} in/s</b><br>Stall acceleration <b>${Math.round(P.aStall/IN)} in/s²</b> · traction limit <b>${Math.round(P.aTrac/IN)} in/s²</b><br>True kS <b>${f2(P.kS)}</b> V · kV <b>${f2(P.kV)}</b> V·s/m · kA <b>${f2(P.kA)}</b> V·s²/m</div>`)}
      ${acc('real','Real-world effects',rl.kVpct!==100||rl.kApct!==100||rl.odoScale||rl.gyroDrift||rl.trackPct!==100?'Mismatch on':'Ideal',`<p class="hint">Change how the simulated robot really behaves without telling the controller. This shows how sensitive each motion is to bad constants and sensor drift.</p><div class="rows">
        ${numRow('Static friction (kS)','cfg.real.kS',rl.kS,'V',0.05,'min="0"')}${numRow('Velocity constant (kV)','cfg.real.kVpct',rl.kVpct,'%',5,'min="50" max="200"')}${numRow('Inertia (kA)','cfg.real.kApct',rl.kApct,'%',5,'min="25" max="300"')}
        ${numRow('Turning inertia factor','cfg.real.ang',rl.ang,'×',0.1,'min="0.2" max="4"')}${numRow('Wheel grip (μ)','cfg.real.mu',rl.mu,'',0.05,'min="0.1" max="2"')}${numRow('Effective track width','cfg.real.trackPct',rl.trackPct,'%',1,'min="80" max="150"')}
        ${numRow('Odometry distance error','cfg.real.odoScale',rl.odoScale,'%',0.5)}${numRow('Gyro drift','cfg.real.gyroDrift',rl.gyroDrift,'°/s',0.1)}</div>`)}
      ${acc('ff','Controller feedforward',C.ffAuto!==false?'Matches robot':'Custom',`<p class="hint">Used by the path controllers (spec stage 3): volts = kS·sign(v) + kV·v + kA·a + kP·(v − measured v), in meters and seconds.</p><div class="rows">
        ${numRow('kS','cfg.ff.kS',ff.kS,'V',0.05,'min="0"')}${numRow('kV','cfg.ff.kV',ff.kV,'V·s/m',0.1,'min="0"')}${numRow('kA','cfg.ff.kA',ff.kA,'V·s²/m',0.05,'min="0"')}${numRow('Velocity kP','cfg.ff.kP',ff.kP,'V·s/m',0.5,'min="0"')}</div>
        <div class="flexrow"><button class="btn" data-act="matchFF" ${C.ffAuto!==false?'disabled':''}>Match the robot model</button></div>`)}
      </div>`}
    else if(state.tab==='tuning'){
      h+=`<div class="v-accordion">
      ${acc('traj','Path speed limits',`${C.traj.v} in/s · ${C.traj.a} in/s²`,`<div class="rows">${numRow('Max wheel speed','cfg.traj.v',C.traj.v,'in/s',1,'min="1"')}${numRow('Max acceleration','cfg.traj.a',C.traj.a,'in/s²',5,'min="1"')}${numRow('Max deceleration','cfg.traj.d',C.traj.d,'in/s²',5,'min="1"')}${numRow('Max lateral accel','cfg.traj.lat',C.traj.lat,'in/s²',5,'min="0"')}</div>
        <p class="hint">The robot's real top speed is ${f1(P.vMax/IN)} in/s. Leave headroom so feedback has voltage left to correct errors.</p>`)}
      ${acc('ram','RAMSETE',`b ${C.ram.b} · ζ ${C.ram.zeta}`,`<div class="rows">${numRow('b','cfg.ram.b',C.ram.b,'',0.1,'min="0"')}${numRow('ζ (zeta)','cfg.ram.zeta',C.ram.zeta,'',0.05,'min="0" max="1"')}</div>`)}
      ${acc('pp','Pure pursuit',`${C.pp.lmin}–${C.pp.lmax} in`,`<div class="rows">${numRow('Min lookahead','cfg.pp.lmin',C.pp.lmin,'in',0.5,'min="1"')}${numRow('Max lookahead','cfg.pp.lmax',C.pp.lmax,'in',0.5,'min="1"')}${numRow('Curvature gain c','cfg.pp.c',C.pp.c,'',0.05,'min="0"')}</div><p class="hint">Lookahead = clamp(c ÷ |curvature|, min, max), so it shrinks on tight curves.</p>`)}
      ${acc('lat','Lateral PID',`kP ${C.lat.kP} · kD ${C.lat.kD}`,`<div class="rows">${numRow('kP','cfg.lat.kP',C.lat.kP,'V/in',0.05,'min="0"')}${numRow('kI','cfg.lat.kI',C.lat.kI,'V/in·s',0.05,'min="0"')}${numRow('kD','cfg.lat.kD',C.lat.kD,'V·s/in',0.01,'min="0"')}${numRow('Settle error','cfg.lat.settle',C.lat.settle,'in',0.25,'min="0"')}${numRow('Settle time','cfg.lat.settleMs',C.lat.settleMs,'ms',10,'min="0"')}</div>`)}
      ${acc('ang','Angular PID',`kP ${C.ang.kP} · kD ${C.ang.kD}`,`<div class="rows">${numRow('kP','cfg.ang.kP',C.ang.kP,'V/°',0.01,'min="0"')}${numRow('kI','cfg.ang.kI',C.ang.kI,'V/°·s',0.01,'min="0"')}${numRow('kD','cfg.ang.kD',C.ang.kD,'V·s/°',0.002,'min="0"')}${numRow('Settle error','cfg.ang.settle',C.ang.settle,'°',0.25,'min="0"')}${numRow('Settle time','cfg.ang.settleMs',C.ang.settleMs,'ms',10,'min="0"')}</div>`)}
      ${acc('misc','Early exit and stopping',`${({coast:'Coast',brake:'Brake',hold:'Hold'})[C.stop]} · glide ${C.ex.glideA} in/s²`,`<div class="rows">
        ${selRow('Stop mode for waits','cfg.stop',C.stop,[['coast','Coast'],['brake','Brake'],['hold','Hold']])}
        ${numRow('Coast drag','cfg.ex.coast',C.ex.coast,'%',5,'min="0" max="100"')}${numRow('Hold strength','cfg.ex.hold',C.ex.hold,'V/in',0.1,'min="0"')}${numRow('Hold damping','cfg.ex.holdD',C.ex.holdD,'V·s/in',0.05,'min="0"')}</div>
        <h4 style="margin-top:var(--space-2)">Guide to target</h4><div class="rows">
        ${numRow('Glide deceleration','cfg.ex.glideA',C.ex.glideA,'in/s²',5,'min="1"')}${numRow('Glide turn deceleration','cfg.ex.glideW',C.ex.glideW,'°/s²',20,'min="1"')}
        ${numRow('Steering gain','cfg.ex.steer',C.ex.steer,'1/s',0.25,'min="0"')}${numRow('Minimum guide speed','cfg.ex.minV',C.ex.minV,'in/s',0.5,'min="0"')}${numRow('Minimum guide turn speed','cfg.ex.minW',C.ex.minW,'°/s',5,'min="0"')}</div>
        <p class="hint">Guide never speeds the robot up beyond how fast it was already going, except up to the minimum speeds so it can finish if it stalls short.</p>`)}
      </div>`}
    else if(state.tab==='saves'){
      h+=`<div><h4>Project JSON</h4><p class="hint">The routine and every setting. Your latest work is also kept in this browser automatically.</p>
        <div class="flexrow" style="margin-top:var(--space-2)"><button class="btn" data-act="jsonOut">Show JSON</button><button class="btn" data-act="jsonDl">Download .json</button><button class="btn" data-act="jsonCopy">Copy JSON</button><label class="btn" for="s-fileIn">Load .json file</label><input type="file" id="s-fileIn" accept=".json,application/json" hidden></div>
        <textarea id="s-jsonBox" placeholder="Paste project JSON here, then load it" style="margin-top:var(--space-3)"></textarea>
        <div class="flexrow" style="margin-top:var(--space-2)"><button class="btn" data-act="jsonIn">Load from text</button><span class="hint" id="s-jsonMsg"></span></div></div>
        <div><h4>Start over</h4><button class="btn ghost danger" data-act="reset">Reset to the example routine</button></div>`}
    const pn=$('#s-panel');pn.innerHTML=h;tagTips(pn);updateFieldHint();
  }
  function updateFieldHint(){const c=selCmd();const t=!c?'Select a command to edit it on the field.':
    c.type==='followPath'?'Click the field to add points to this path. Drag points and square handles to shape it; hold Alt to move one handle on its own.':
    c.type==='turnHeading'?'Click the field to aim this turn at that spot.':c.type==='wait'?'Wait has nothing to place on the field.':
    c.type==='moveToPose'?'Click the field to place the target, then drag the round knob to set the end heading.':'Click the field or drag the marker to place the target.';$('#s-fieldHint').textContent=t}

  /* ---------------- Editing ---------------- */
  function setPath(obj,path,val){const k=path.split('.');let o=obj;for(let i=0;i<k.length-1;i++)o=o[k[i]];o[k[k.length-1]]=val}
  function bindSet(el){const b=el.dataset.bind,raw=el.type==='checkbox'?el.checked:el.value,num=parseFloat(raw),cmd=selCmd();
    if(b.startsWith('cmd.')){if(!cmd)return;const key=b.slice(4),cur=cmd[key];if(typeof cur==='boolean')cmd[key]=raw;else if(typeof cur==='number'){if(isFinite(num))cmd[key]=key==='h'?num:num}else cmd[key]=raw;return}
    if(b.startsWith('anc.')){const[,k,f]=b.split('.');const a=cmd&&cmd.anchors[+k];if(a&&isFinite(num))a[f]=clamp(num,-HALF,HALF);return}
    if(b.startsWith('start.')){if(isFinite(num))proj.start[b.slice(6)]=b==='start.h'?num:clamp(num,-HALF,HALF);return}
    if(b==='cfg.robot.size'){if(raw!=='c'){proj.cfg.robot.w=proj.cfg.robot.l=+raw}return}
    if(b.startsWith('cfg.ff.')){if(isFinite(num)){proj.cfg.ff[b.slice(7)]=num;if(b!=='cfg.ff.kP')proj.cfg.ffAuto=false}return}
    if(b==='cfg.stop'){proj.cfg.stop=raw;return}
    if(b==='cfg.drive.cart'){proj.cfg.drive.cart=+raw;return}
    if(isFinite(num))setPath(proj,b,num);
  }
  let rq=0;const schedule=(panel)=>{if(rq)return;rq=requestAnimationFrame(()=>{rq=0;resim();drawAll();if(panel)renderPanel()})};
  const panel=$('#s-panel');
  panel.addEventListener('input',e=>{const el=e.target;if(!el.dataset.bind||el.tagName==='SELECT'||el.type==='checkbox')return;bindSet(el);schedule(false)});
  panel.addEventListener('change',e=>{const el=e.target;if(el.id==='s-fileIn'){readFile(el);return}if(!el.dataset.bind)return;bindSet(el);resim();drawAll();renderPanel()});
  function defaultsAt(type,pose){const a=pose.h*D2R,f=(d,off=0)=>({x:clamp(pose.x+Math.sin(a+off)*d,-66,66),y:clamp(pose.y+Math.cos(a+off)*d,-66,66)});
    const base=JSON.parse(JSON.stringify(CMDS[type].def));
    if(type==='moveToPoint'||type==='turnPoint')Object.assign(base,type==='turnPoint'?f(24,Math.PI/2):f(24));
    if(type==='moveToPose')Object.assign(base,f(30),{h:Math.round(norm360(pose.h))});
    if(type==='turnHeading')base.h=Math.round(norm360(pose.h+90));
    if(type==='followPath'){const p1=f(36,0.35),ux=Math.sin(a),uy=Math.cos(a);base.anchors=[{x:pose.x,y:pose.y,ix:0,iy:0,ox:ux*14,oy:uy*14},{x:p1.x,y:p1.y,ix:-ux*12,iy:-uy*12,ox:ux*12,oy:uy*12}]}
    return{id:nid(),type,...base}}
  const endPoseOf=i=>i<0?proj.start:R.res[i]?R.res[i].endPose:proj.start;
  panel.addEventListener('click',e=>{const t=e.target.closest('.v-acc-trigger');if(t){const it=t.closest('.v-acc-item'),open=it.getAttribute('data-open')!=='true';it.setAttribute('data-open',String(open));t.setAttribute('aria-expanded',String(open));state.acc[t.dataset.acc]=open;return}
    const b=e.target.closest('[data-act]');if(!b)return;const act=b.dataset.act,cmd=selCmd();
    switch(act){
      case'sel':{const i=+b.dataset.i;state.sel=i;state.selAnchor=null;const r=R.res[i];if(r&&!state.playing)state.time=r.t0;break}
      case'add':{const type=$('#s-newType').value,at=proj.cmds.length?state.sel+1:0,pose=endPoseOf(at-1);proj.cmds.splice(at,0,defaultsAt(type,pose));state.sel=at;state.selAnchor=null;break}
      case'up':if(state.sel>0){const c=proj.cmds.splice(state.sel,1)[0];proj.cmds.splice(state.sel-1,0,c);state.sel--}break;
      case'down':if(state.sel<proj.cmds.length-1){const c=proj.cmds.splice(state.sel,1)[0];proj.cmds.splice(state.sel+1,0,c);state.sel++}break;
      case'dup':if(cmd){const c=JSON.parse(JSON.stringify(cmd));c.id=nid();proj.cmds.splice(state.sel+1,0,c);state.sel++}break;
      case'del':if(cmd){confirmDelete(`Delete command ${state.sel+1}?`,`${CMDS[cmd.type].name} · ${summary(cmd)} will be removed from the routine.`,()=>{proj.cmds.splice(state.sel,1);state.sel=Math.min(state.sel,proj.cmds.length-1);state.selAnchor=null;resim();drawAll();renderPanel()})}return;
      case'delAnc':{const k=+b.dataset.k;confirmDelete(`Remove path point ${k}?`,'The path will be redrawn without it.',()=>{cmd.anchors.splice(k,1);state.selAnchor=null;resim();drawAll();renderPanel()});return}
      case'snapStart':if(cmd&&cmd.anchors.length){const sp=startOf(state.sel),a0=cmd.anchors[0],len=Math.max(6,Math.hypot(a0.ox,a0.oy)),dir=(sp.h+(cmd.rev?180:0))*D2R;Object.assign(a0,{x:sp.x,y:sp.y,ox:Math.sin(dir)*len,oy:Math.cos(dir)*len})}break;
      case'matchFF':proj.cfg.ffAuto=true;break;
      case'jsonOut':$('#s-jsonBox').value=JSON.stringify(proj,null,1);return;
      case'jsonCopy':{const txt=JSON.stringify(proj);$('#s-jsonBox').value=txt;const ok=()=>$('#s-jsonMsg').textContent='Copied.',fb=()=>{$('#s-jsonBox').select();$('#s-jsonMsg').textContent='Copy is blocked here. The JSON is selected; press Ctrl or Cmd+C.'};try{navigator.clipboard.writeText(txt).then(ok,fb)}catch(_){fb()}return}
      case'jsonIn':loadJSON($('#s-jsonBox').value);return;
      case'jsonDl':downloadBlob(new Blob([JSON.stringify(proj,null,1)],{type:'application/json'}),`tank-sim-${new Date().toISOString().slice(0,10)}.json`);return;
      case'reset':confirmDelete('Reset to the example routine?','Your commands and settings will be replaced by the example.',()=>{proj=exampleProject();state.sel=0;state.selAnchor=null;resim();state.time=R.T;drawAll();renderPanel()});return;
    }
    resim();drawAll();renderPanel();
  });
  function loadExternal(p){proj=normalizeProject(p);state.sel=0;state.selAnchor=null;state.tab='routine';resim();state.time=R.T;renderPanel();drawAll()}
  function loadJSON(txt){try{const p=JSON.parse(txt);if(!validProject(p))throw new Error('It is missing the start pose, settings or commands.');p.cmds.forEach(c=>{if(!CMDS[c.type])throw new Error(`Unknown command type "${c.type}".`);c.id=c.id||nid()});proj=normalizeProject(p);state.sel=0;state.selAnchor=null;resim();state.time=R.T;drawAll();renderPanel();const m=$('#s-jsonMsg');if(m)m.textContent='Project loaded.'}catch(err){const m=$('#s-jsonMsg');if(m)m.textContent='Could not load it: '+err.message}}
  function readFile(el){const f=el.files&&el.files[0];if(!f)return;const r=new FileReader();r.onload=()=>{$('#s-jsonBox').value=r.result;loadJSON(r.result)};r.readAsText(f)}

  /* ---------------- Field interaction ---------------- */
  let drag=null;const snapV=v=>state.snap?Math.round(v/state.snap)*state.snap:v;
  const evPt=e=>{const r=fieldCv.getBoundingClientRect();return{X:e.clientX-r.left,Y:e.clientY-r.top}};
  function hitTest(X,Y){const near=(x,y,r=10)=>Math.hypot(px(x)-X,py(y)-Y)<=r,c=selCmd();
    if(c&&c.type==='followPath'){const A=c.anchors;for(let k=A.length-1;k>=0;k--){const a=A[k];if(k<A.length-1&&near(a.x+a.ox,a.y+a.oy,8))return{t:'hout',k};if(k>0&&near(a.x+a.ix,a.y+a.iy,8))return{t:'hin',k}}for(let k=A.length-1;k>=0;k--)if(near(A[k].x,A[k].y,10))return{t:'anc',k}}
    if(c&&c.type==='moveToPose'){const k=knobOf(c);if(near(k.x,k.y,9))return{t:'knob'}}
    if(c&&'x' in c&&near(c.x,c.y,12))return{t:'tgt'};
    const sk=startKnob();if(near(sk.x,sk.y,8))return{t:'sknob'};
    if(state.showAll)for(let i=proj.cmds.length-1;i>=0;i--){if(i===state.sel)continue;const o=proj.cmds[i];if('x' in o&&near(o.x,o.y,11))return{t:'pick',i};if(o.type==='followPath'&&o.anchors.some(a=>near(a.x,a.y,8)))return{t:'pick',i}}
    const s=proj.start,dx=wx(X)-s.x,dy=wy(Y)-s.y,a=s.h*D2R,lf=dx*Math.sin(a)+dy*Math.cos(a),lr=dx*Math.cos(a)-dy*Math.sin(a);if(Math.abs(lf)<proj.cfg.robot.l/2&&Math.abs(lr)<proj.cfg.robot.w/2)return{t:'start'};
    return null}
  function emptyClick(x,y,e){const c=selCmd();if(!c)return false;const sx=clamp(snapV(x),-HALF,HALF),sy=clamp(snapV(y),-HALF,HALF);
    if(c.type==='followPath'){const A=c.anchors;if(!A.length){const sp=startOf(state.sel),d=(sp.h+(c.rev?180:0))*D2R;A.push({x:sp.x,y:sp.y,ix:0,iy:0,ox:Math.sin(d)*12,oy:Math.cos(d)*12})}
      const p=A[A.length-1],dx=sx-p.x,dy=sy-p.y;if(Math.hypot(dx,dy)<2)return false;if(Math.hypot(p.ox,p.oy)<0.01){p.ox=dx/3;p.oy=dy/3}
      A.push({x:sx,y:sy,ix:-dx/3,iy:-dy/3,ox:dx/3,oy:dy/3});state.selAnchor=A.length-1;return true}
    if(c.type==='turnHeading'){const sp=startOf(state.sel);let hh=bearing(sp.x,sp.y,x,y);if(!e.shiftKey)hh=Math.round(hh/15)*15;c.h=norm360(hh);return true}
    if('x' in c){c.x=sx;c.y=sy;return true}return false}
  fieldCv.addEventListener('pointerdown',e=>{const{X,Y}=evPt(e),h=hitTest(X,Y);
    if(h&&h.t==='pick'){state.sel=h.i;state.selAnchor=null;const r=R.res[h.i];if(r&&!state.playing)state.time=r.t0;drawAll();renderPanel();return}
    if(h){if(h.t==='anc')state.selAnchor=h.k;fieldCv.setPointerCapture(e.pointerId);drag={...h,moved:false};drawAll();return}
    if(emptyClick(wx(X),wy(Y),e)){resim();drawAll();renderPanel()}});
  fieldCv.addEventListener('pointermove',e=>{const{X,Y}=evPt(e),x=wx(X),y=wy(Y);
    $('#s-hud').textContent=Math.abs(x)<=HALF&&Math.abs(y)<=HALF?`(${f1(x)}, ${f1(y)}) in`:'—';
    if(!drag){const h=hitTest(X,Y);fieldCv.style.cursor=h?(h.t==='sknob'||h.t==='knob'?'grab':h.t==='pick'?'pointer':'move'):'crosshair';return}
    drag.moved=true;const c=selCmd(),sx=clamp(snapV(x),-HALF,HALF),sy=clamp(snapV(y),-HALF,HALF);
    if(drag.t==='anc'){const a=c.anchors[drag.k];a.x=sx;a.y=sy}
    else if(drag.t==='hout'||drag.t==='hin'){const a=c.anchors[drag.k],dx=x-a.x,dy=y-a.y;if(drag.t==='hout'){a.ox=dx;a.oy=dy}else{a.ix=dx;a.iy=dy}
      if(!e.altKey){const len=drag.t==='hout'?Math.hypot(a.ix,a.iy):Math.hypot(a.ox,a.oy),d=Math.hypot(dx,dy)||1;if(len>0.01){if(drag.t==='hout'){a.ix=-dx/d*len;a.iy=-dy/d*len}else{a.ox=-dx/d*len;a.oy=-dy/d*len}}}}
    else if(drag.t==='tgt'){c.x=sx;c.y=sy}
    else if(drag.t==='knob'){let hh=bearing(c.x,c.y,x,y);if(!e.shiftKey)hh=Math.round(hh/15)*15;c.h=norm360(hh)}
    else if(drag.t==='sknob'){let hh=bearing(proj.start.x,proj.start.y,x,y);if(!e.shiftKey)hh=Math.round(hh/15)*15;proj.start.h=norm360(hh)}
    else if(drag.t==='start'){proj.start.x=sx;proj.start.y=sy}
    schedule(false)});
  const endDrag=()=>{if(drag){const m=drag.moved;drag=null;if(m){resim();drawAll();renderPanel()}}};
  fieldCv.addEventListener('pointerup',endDrag);fieldCv.addEventListener('pointercancel',endDrag);fieldCv.addEventListener('pointerleave',()=>{$('#s-hud').textContent='—'});

  /* Chart scrubbing */
  let scrub=false;const scrubTo=e=>{const r=chartCv.getBoundingClientRect(),X=e.clientX-r.left;state.time=clamp((X-CH.pl)/(r.width-CH.pl-CH.pr)*R.T,0,R.T);state.playing=false;drawAll()};
  chartCv.addEventListener('pointerdown',e=>{scrub=true;chartCv.setPointerCapture(e.pointerId);scrubTo(e)});chartCv.addEventListener('pointermove',e=>{if(scrub)scrubTo(e)});chartCv.addEventListener('pointerup',()=>{scrub=false});

  /* ---------------- Toolbar & playback ---------------- */
  $('#s-tabs').onclick=e=>{const b=e.target.closest('button');if(!b)return;state.tab=b.dataset.tab;renderPanel()};
  $('#s-chartSeg').onclick=e=>{const b=e.target.closest('button');if(!b)return;state.chart=b.dataset.chart;syncSegs();drawChart()};
  $('#s-snapSel').onchange=e=>{state.snap=+e.target.value};
  $('#s-showOdom').onchange=e=>{state.showOdom=e.target.checked;drawScene()};
  $('#s-showAll').onchange=e=>{state.showAll=e.target.checked;drawScene()};
  $('#s-rateSel').onchange=e=>{state.rate=+e.target.value};
  $('#s-restartBtn').onclick=()=>{state.time=0;drawAll()};
  $('#s-playBtn').onclick=()=>togglePlay();
  function syncSegs(){root.querySelectorAll('#s-chartSeg button').forEach(b=>b.setAttribute('aria-pressed',b.dataset.chart===state.chart));$('#s-snapSel').value=state.snap}
  let last=0;function togglePlay(){state.playing=!state.playing;if(state.playing){if(state.time>=R.T-0.01)state.time=0;last=performance.now();requestAnimationFrame(tick)}updateTransport()}
  function tick(now){if(!state.playing)return;const dt=Math.min(0.1,(now-last)/1000);last=now;state.time+=dt*state.rate;if(state.time>=R.T){state.time=R.T;state.playing=false}drawAll();if(state.playing)requestAnimationFrame(tick)}
  document.addEventListener('keydown',e=>{if(!hooks.isActive()||/INPUT|SELECT|TEXTAREA/.test(e.target.tagName)||isModalOpen())return;const k=e.key;
    if(k===' '){e.preventDefault();togglePlay()}
    else if(k==='ArrowRight'||k==='ArrowLeft'){state.time=clamp(state.time+(k==='ArrowRight'?1:-1)*(e.shiftKey?1:0.05),0,R.T);drawAll()}
    else if(k==='Delete'||k==='Backspace'){const c=selCmd();if(!c)return;e.preventDefault();
      if(c.type==='followPath'&&state.selAnchor!=null){const kk=state.selAnchor;confirmDelete(`Remove path point ${kk}?`,'The path will be redrawn without it.',()=>{c.anchors.splice(kk,1);state.selAnchor=null;resim();drawAll();renderPanel()})}
      else confirmDelete(`Delete command ${state.sel+1}?`,`${CMDS[c.type].name} · ${summary(c)} will be removed from the routine.`,()=>{proj.cmds.splice(state.sel,1);state.sel=Math.min(state.sel,proj.cmds.length-1);resim();drawAll();renderPanel()})}});

  /* ---------------- Hover help ---------------- */
  const TIPS={
    'start.x':'Where the robot\'s center starts, in inches from the field center. Negative X is the red side.','start.y':'Starting Y, in inches from the field center. Positive Y is toward the top wall.','start.h':'Starting heading. 0° faces the top wall, 90° faces the blue side, clockwise positive.',
    'cmd.x':'Target X in inches from the field center.','cmd.y':'Target Y in inches from the field center.','cmd.h':'Target heading in compass degrees: 0° faces the top wall, clockwise positive.',
    'cmd.back':'Drive or face the target with the back of the robot.','cmd.rev':'Follow the path driving backwards. The robot faces opposite to the direction of travel.',
    'cmd.maxSpd':'Caps motor output for this command, as a percent of 12 V.','cmd.minSpd':'Keeps at least this much output until the early exit range, for chaining motions without stopping. 0 means stop and settle.',
    'cmd.exit':'End the command once the robot is this close to the target (inches, or degrees for turns), instead of waiting to settle. 0 turns early exit off. A min speed above 0 also exits once the robot passes the target.',
    'cmd.exitMode':'What the drive does after an early exit. <b>Chain</b> starts the next command right away. <b>Coast</b> lets the robot roll. <b>Brake</b> shorts the motors to stop hard. <b>Guide</b> keeps steering at the target and slows along a glide profile so it drifts in.',
    'cmd.exitMs':'Longest the exit behavior runs before the next command starts. Coast, brake and guide also end early once the robot stops or arrives.',
    'cmd.timeout':'The command gives up after this long. A timeout on a real robot usually means a gain or a target is off. For paths, 0 means the profiled time plus 1.5 s.',
    'cmd.dir':'Which way to turn. Shortest picks the smaller rotation.','cmd.lead':'Boomerang lead. The carrot point sits this fraction of the remaining distance behind the target, along its heading. Higher values make wider, more curved approaches.',
    'cmd.ctrl':'<b>RAMSETE</b> tracks a time-stamped trajectory and corrects position and heading errors. <b>Pure pursuit</b> steers toward a lookahead point and ignores timing.',
    'cmd.spd':'Scales the path speed limit for this path only.','cmd.tol':'How close the robot must end to the last path point to count as settled.','cmd.ms':'How long to wait. The drive uses the stop mode from Tuning.',
    'cfg.robot.size':'Robot footprint for drawing and the start-pose drag area.','cfg.robot.w':'Footprint width.','cfg.robot.l':'Footprint length.',
    'cfg.drive.wheel':'Drive wheel diameter.','cfg.drive.cart':'V5 motor cartridge free speed.','cfg.drive.gIn':'Teeth on the motor gear. Wheel rpm = cartridge rpm × motor teeth ÷ wheel teeth.','cfg.drive.gOut':'Teeth on the wheel gear.',
    'cfg.drive.motors':'Total drive motors, half on each side. More motors raise acceleration, not top speed.','cfg.drive.mass':'Robot weight. Heavier robots accelerate more slowly.','cfg.drive.eff':'Share of free speed and torque the drivetrain really delivers. 70–85% is typical.',
    'cfg.drive.track':'Distance between left and right wheel centers. The controllers use this value.',
    'cfg.real.kS':'Voltage needed to start moving. Below this, a stopped side stays put. This is why P-only turns can stop short.','cfg.real.kVpct':'How the real kV compares to the model. Above 100% the robot is slower than the controller expects.',
    'cfg.real.kApct':'How the real kA compares to the model. Above 100% the robot is heavier or more sluggish than expected.','cfg.real.ang':'Extra inertia when turning compared to driving straight. Long or heavy robots are higher.',
    'cfg.real.mu':'Wheel grip. Caps wheel acceleration at μ·g; above that the wheels slip.','cfg.real.trackPct':'Real turning track width compared to the measured one. Skid steering usually acts wider than the wheels.',
    'cfg.real.odoScale':'Odometry overestimates distance by this percent. The controller trusts odometry, so the real robot ends up short or long.','cfg.real.gyroDrift':'Heading drift of the inertial sensor in degrees per second, clockwise positive.',
    'cfg.ff.kS':'Feedforward static voltage, added in the direction of motion.','cfg.ff.kV':'Feedforward volts per meter per second of wheel speed.','cfg.ff.kA':'Feedforward volts per meter per second squared of wheel acceleration.','cfg.ff.kP':'Feedback on wheel velocity error, in volts per meter per second.',
    'cfg.traj.v':'Fastest a wheel may go on a path. Curves lower the robot speed so the outer wheel stays under this.','cfg.traj.a':'Fastest the path speeds up.','cfg.traj.d':'Fastest the path slows down.','cfg.traj.lat':'Sideways acceleration limit on curves: speed ≤ √(limit ÷ curvature). 0 turns it off.',
    'cfg.ram.b':'RAMSETE aggressiveness, like a proportional gain. 2 is the usual default.','cfg.ram.zeta':'RAMSETE damping, between 0 and 1. 0.7 is the usual default.',
    'cfg.pp.lmin':'Shortest lookahead, used on tight curves.','cfg.pp.lmax':'Longest lookahead, used on straights.','cfg.pp.c':'Lookahead = c ÷ |curvature|, clamped between min and max.',
    'cfg.lat.kP':'Volts per inch of distance error for move commands.','cfg.lat.kI':'Integral gain. Helps finish the last fraction of an inch against friction. Resets when the error changes sign.','cfg.lat.kD':'Damping: volts per inch per second.','cfg.lat.settle':'Error that counts as arrived.','cfg.lat.settleMs':'How long the error must stay inside the settle range.',
    'cfg.ang.kP':'Volts per degree of heading error.','cfg.ang.kI':'Integral gain for turns.','cfg.ang.kD':'Damping: volts per degree per second.','cfg.ang.settle':'Heading error that counts as arrived.','cfg.ang.settleMs':'How long the heading must stay inside the settle range.',
    'cfg.stop':'What the drive does during waits and after the routine, like the V5 brake modes. <b>Coast</b> rolls freely, <b>brake</b> shorts the motors, <b>hold</b> actively holds wheel position.','cfg.ex.coast':'Drivetrain drag while coasting, as a percent of the motor kV term. Higher means the robot rolls a shorter distance.','cfg.ex.hold':'Hold mode: volts per inch the wheels are pushed away from where they stopped.','cfg.ex.holdD':'Hold mode damping: volts per inch per second of wheel speed.','cfg.ex.glideA':'Guide mode slows the robot so it could just stop at the target: speed ≤ √(2 × this × distance left). Lower values brake earlier and gentler.','cfg.ex.glideW':'The same glide limit for turns, in degrees per second squared.','cfg.ex.steer':'Guide mode steering: degrees per second of turn for each degree the target is off to the side.','cfg.ex.minV':'Guide mode keeps at least this speed so it can finish if the robot stops short, and backs up at no more than this if it overshoots.','cfg.ex.minW':'Minimum turn speed in guide mode for turns.',
    snap:'Round dragged and clicked points to this grid. A tile is 24 in.',odom:'Show where odometry thinks the robot is when it disagrees with the true position.',all:'Show every command\'s targets, not only the selected one.',
    run:'Play the routine (Space). Arrow keys step through time; hold Shift for 1 s steps.',restart:'Jump back to 0 s.',rate:'Playback speed.',newType:'Command to add after the selected one.',
  };
  [['#s-snapSel','snap'],['label[for=s-snapSel]','snap'],['label[for=s-showOdom]','odom'],['label[for=s-showAll]','all'],['#s-playBtn','run'],['#s-restartBtn','restart'],['#s-rateSel','rate']].forEach(([s,k])=>{const el=root.querySelector(s);if(el)el.dataset.tip=k});
  function tagTips(root){root.querySelectorAll('[data-bind]').forEach(el=>{const key=el.dataset.bind;if(!TIPS[key])return;el.dataset.tip=key;const lb=el.id&&root.querySelector(`label[for="${el.id}"]`);if(lb)lb.dataset.tip=key});const nt=root.querySelector('#s-newType');if(nt)nt.dataset.tip='newType'}
  registerTips('sim',TIPS);
  onThemeChange(()=>drawAll());

  /* ---------------- Boot ---------------- */
  resim();state.time=R.T;syncSegs();renderPanel();drawAll();
  new ResizeObserver(()=>{drawScene();drawChart()}).observe(root.querySelector('.fieldwrap'));
  new ResizeObserver(()=>drawChart()).observe(chartCv);
  if(document.fonts&&document.fonts.ready)document.fonts.ready.then(()=>drawAll());

  return{
    /** Called by the shell when this view becomes visible. */
    show(){syncSegs();renderPanel();drawAll()},
    /** Replace the routine, e.g. with one converted from a planner route (see bridge.js). */
    loadProject:loadExternal,
  };
}
