"""Live training dashboard: serves Saved/NavRuns/<run>/metrics.jsonl and a self-refreshing page on http://localhost:8770.

    python serve_dashboard.py [--run clearance] [--port 8770]
The page polls the metrics file every 2 s and redraws the curves; nothing to install (Chart.js comes from a CDN).
"""
import argparse
import functools
import http.server
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>ClearanceNet training</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
<style>
:root{color-scheme:dark light;--bg:#0f1115;--fg:#e8eaf0;--card:#181b22;--mut:#8b93a7;--a:#4ea1ff;--b:#ff9f43;--g:#3ddc97}
@media (prefers-color-scheme:light){:root{--bg:#f4f5f8;--fg:#1a1d24;--card:#fff;--mut:#5b6478}}
body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
header{padding:14px 20px;display:flex;gap:18px;align-items:baseline;flex-wrap:wrap}
h1{font-size:18px;margin:0}.mut{color:var(--mut)}
.kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px;padding:0 20px 10px}
.k{background:var(--card);border-radius:10px;padding:10px 14px}.k b{display:block;font-size:22px;margin-top:2px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:12px;padding:0 20px 20px}
.c{background:var(--card);border-radius:10px;padding:12px}.c h3{margin:0 0 6px;font-size:13px;color:var(--mut);font-weight:600}
canvas{max-height:260px}
</style></head><body>
<header><h1>ClearanceNet training</h1><span class="mut" id="status">waiting for metrics...</span></header>
<div class="kpis">
<div class="k"><span class="mut">epoch</span><b id="kEpoch">-</b></div>
<div class="k"><span class="mut">clearance MAE (cm)</span><b id="kMae">-</b><span class="mut" id="kMaeB"></span></div>
<div class="k"><span class="mut">close floor-object recall</span><b id="kRec">-</b><span class="mut" id="kRecB"></span></div>
<div class="k"><span class="mut">false alarms</span><b id="kFa">-</b><span class="mut" id="kFaB"></span></div>
<div class="k"><span class="mut">elapsed</span><b id="kEl">-</b></div>
</div>
<div class="grid">
<div class="c"><h3>train loss (per 20 steps)</h3><canvas id="cLoss"></canvas></div>
<div class="c"><h3>clearance error, bins &lt; 15 m (cm) - lower is better</h3><canvas id="cMae"></canvas></div>
<div class="c"><h3>close floor-object recall, 1-4 m (%) - target 95</h3><canvas id="cRec"></canvas></div>
<div class="c"><h3>false alarms: free bearings reported &lt; 3 m (%)</h3><canvas id="cFa"></canvas></div>
<div class="c"><h3>REAL UE scans: clearance error (cm)</h3><canvas id="cUeMae"></canvas></div>
<div class="c"><h3>REAL UE scans: objects 1-4 m caught (%) / false alarms (%)</h3><canvas id="cUeRec"></canvas></div>
</div>
<script>
const css=getComputedStyle(document.documentElement);
function mk(id,ds,log){return new Chart(document.getElementById(id),{type:'line',data:{labels:[],datasets:ds},
 options:{animation:false,responsive:true,elements:{point:{radius:0},line:{borderWidth:2}},
 scales:{x:{ticks:{color:css.getPropertyValue('--mut')}},y:{type:log?'logarithmic':'linear',ticks:{color:css.getPropertyValue('--mut')}}},
 plugins:{legend:{labels:{color:css.getPropertyValue('--fg')}}}}});}
const A='#4ea1ff',B='#ff9f43';
const cLoss=mk('cLoss',[{label:'loss',data:[],borderColor:A}],true);
const cMae=mk('cMae',[{label:'ClearanceNet',data:[],borderColor:A},{label:'height-filter baseline',data:[],borderColor:B,borderDash:[6,4]}]);
const cRec=mk('cRec',[{label:'ClearanceNet',data:[],borderColor:A},{label:'baseline',data:[],borderColor:B,borderDash:[6,4]}]);
const cFa=mk('cFa',[{label:'ClearanceNet',data:[],borderColor:A},{label:'baseline',data:[],borderColor:B,borderDash:[6,4]}]);
const cUeMae=mk('cUeMae',[{label:'ClearanceNet',data:[],borderColor:A},{label:'baseline',data:[],borderColor:B,borderDash:[6,4]}]);
const cUeRec=mk('cUeRec',[{label:'caught %',data:[],borderColor:'#3ddc97'},{label:'baseline caught %',data:[],borderColor:B,borderDash:[6,4]},{label:'false alarms %',data:[],borderColor:'#ff5d5d'}]);
function set(c,labels,...series){c.data.labels=labels;series.forEach((s,i)=>c.data.datasets[i].data=s);c.update();}
async function tick(){
 try{
  const t=await (await fetch('metrics.jsonl?'+Date.now())).text();
  const rows=t.trim().split('\\n').filter(Boolean).map(l=>JSON.parse(l));
  const st=rows.filter(r=>r.type==='step'),ep=rows.filter(r=>r.type==='epoch'),done=rows.find(r=>r.type==='done'),start=rows.find(r=>r.type==='start');
  set(cLoss,st.map(r=>r.step),st.map(r=>r.loss));
  const e=ep.map(r=>r.epoch);
  set(cMae,e,ep.map(r=>r.mae),ep.map(r=>r.base_mae));
  set(cRec,e,ep.map(r=>100*r.recall),ep.map(r=>100*r.base_recall));
  set(cFa,e,ep.map(r=>100*r.false_alarm),ep.map(r=>100*r.base_false_alarm));
  const eu=ep.filter(r=>r.ue_mae!==undefined);
  set(cUeMae,eu.map(r=>r.epoch),eu.map(r=>r.ue_mae),eu.map(r=>r.ue_base_mae));
  set(cUeRec,eu.map(r=>r.epoch),eu.map(r=>100*r.ue_recall),eu.map(r=>100*r.ue_base_recall),eu.map(r=>100*r.ue_fa));
  if(ep.length){const l=ep[ep.length-1];
   kEpoch.textContent=l.epoch+(start?' / '+start.epochs:'');kMae.textContent=l.mae.toFixed(1);kMaeB.textContent='baseline '+l.base_mae.toFixed(1);
   kRec.textContent=(100*l.recall).toFixed(1)+'%';kRecB.textContent='baseline '+(100*l.base_recall).toFixed(1)+'%';
   kFa.textContent=(100*l.false_alarm).toFixed(1)+'%';kFaB.textContent='baseline '+(100*l.base_false_alarm).toFixed(1)+'%';
   kEl.textContent=Math.round(l.elapsed)+' s';}
  document.getElementById('status').textContent=done?'finished - test set scored':(start?('training on '+start.device+', '+start.params+' parameters'):'waiting...');
 }catch(e){document.getElementById('status').textContent='waiting for metrics...';}
}
tick();setInterval(tick,2000);
</script></body></html>"""


PAGE_POLICY = """<!doctype html><html><head><meta charset="utf-8"><title>Nav policy training</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
<style>
:root{color-scheme:dark light;--bg:#0f1115;--fg:#e8eaf0;--card:#181b22;--mut:#8b93a7}
@media (prefers-color-scheme:light){:root{--bg:#f4f5f8;--fg:#1a1d24;--card:#fff;--mut:#5b6478}}
body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
header{padding:14px 20px;display:flex;gap:18px;align-items:baseline;flex-wrap:wrap}h1{font-size:18px;margin:0}.mut{color:var(--mut)}
.kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px;padding:0 20px 10px}
.k{background:var(--card);border-radius:10px;padding:10px 14px}.k b{display:block;font-size:22px;margin-top:2px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:12px;padding:0 20px 20px}
.c{background:var(--card);border-radius:10px;padding:12px}.c h3{margin:0 0 6px;font-size:13px;color:var(--mut);font-weight:600}canvas{max-height:260px}
</style></head><body>
<header><h1>Navigation policy (PPO)</h1><span class="mut" id="status">waiting for metrics...</span></header>
<div class="kpis">
<div class="k"><span class="mut">curriculum level</span><b id="kLv">-</b></div>
<div class="k"><span class="mut">success (recent episodes)</span><b id="kS">-</b><span class="mut" id="kSb"></span></div>
<div class="k"><span class="mut">collisions</span><b id="kC">-</b><span class="mut" id="kCb"></span></div>
<div class="k"><span class="mut">time to goal</span><b id="kT">-</b><span class="mut" id="kTb"></span></div>
<div class="k"><span class="mut">env steps</span><b id="kN">-</b><span class="mut" id="kF"></span></div>
</div>
<div class="grid">
<div class="c"><h3>success rate (%) - dashed = classical controller on the same level</h3><canvas id="c1"></canvas></div>
<div class="c"><h3>collision / timeout rate (%)</h3><canvas id="c2"></canvas></div>
<div class="c"><h3>mean reward per step</h3><canvas id="c3"></canvas></div>
<div class="c"><h3>curriculum level and time to goal (s)</h3><canvas id="c4"></canvas></div>
</div>
<script>
const css=getComputedStyle(document.documentElement);const M=css.getPropertyValue('--mut'),Fg=css.getPropertyValue('--fg');
function mk(id,ds){return new Chart(document.getElementById(id),{type:'line',data:{labels:[],datasets:ds},options:{animation:false,elements:{point:{radius:0},line:{borderWidth:2}},
 scales:{x:{ticks:{color:M}},y:{ticks:{color:M}},y2:{position:'right',ticks:{color:M},grid:{drawOnChartArea:false},display:ds.some(d=>d.yAxisID==='y2')}},plugins:{legend:{labels:{color:Fg}}}}});}
const A='#4ea1ff',B='#ff9f43',G='#3ddc97',Rd='#ff5d5d';
const c1=mk('c1',[{label:'learned',data:[],borderColor:A},{label:'classical',data:[],borderColor:B,borderDash:[6,4]}]);
const c2=mk('c2',[{label:'collision',data:[],borderColor:Rd},{label:'timeout',data:[],borderColor:B},{label:'classical collision',data:[],borderColor:Rd,borderDash:[6,4]}]);
const c3=mk('c3',[{label:'reward',data:[],borderColor:G}]);
const c4=mk('c4',[{label:'level',data:[],borderColor:A,stepped:true},{label:'time to goal (s)',data:[],borderColor:G,yAxisID:'y2'}]);
function set(c,l,...s){c.data.labels=l;s.forEach((v,i)=>c.data.datasets[i].data=v);c.update();}
async function tick(){try{
 const t=await (await fetch('metrics.jsonl?'+Date.now())).text();
 const rows=t.trim().split(String.fromCharCode(10)).filter(Boolean).map(l=>JSON.parse(l));
 const it=rows.filter(r=>r.type==='iter'),st=rows.find(r=>r.type==='start'),done=rows.find(r=>r.type==='done');
 const x=it.map(r=>r.iter);
 set(c1,x,it.map(r=>100*r.success),it.map(r=>100*r.base_success));
 set(c2,x,it.map(r=>100*r.collision),it.map(r=>100*r.timeout),it.map(r=>100*r.base_collision));
 set(c3,x,it.map(r=>r.reward));
 set(c4,x,it.map(r=>r.level),it.map(r=>r.time));
 if(it.length){const l=it[it.length-1];
  kLv.textContent=l.level+' / '+((st?st.levels:4)-1);kS.textContent=(100*l.success).toFixed(0)+'%';kSb.textContent='classical '+(100*l.base_success).toFixed(0)+'%';
  kC.textContent=(100*l.collision).toFixed(0)+'%';kCb.textContent='classical '+(100*l.base_collision).toFixed(0)+'%';
  kT.textContent=l.time==null?'-':l.time.toFixed(1)+' s';kTb.textContent='classical '+(l.base_time==null?'-':l.base_time.toFixed(1)+' s');
  kN.textContent=(l.steps/1e6).toFixed(2)+' M';kF.textContent=l.fps+' steps/s';}
 document.getElementById('status').textContent=done?'finished':(st?'training, '+st.envs+' parallel envs':'waiting...');
}catch(e){document.getElementById('status').textContent='waiting for metrics...';}}
tick();setInterval(tick,2000);
</script></body></html>"""

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, run_dir=None, page=None, **k):
        self.run_dir = run_dir
        self.page = page or PAGE
        super().__init__(*a, directory=run_dir, **k)

    def do_GET(self):
        if self.path.split("?")[0] in ("/", "/index.html"):
            body = self.page.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="clearance")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--kind", default="clearance", choices=["clearance", "policy"])
    a = ap.parse_args()
    run_dir = os.path.join(ROOT, "Saved", "NavRuns", a.run)
    os.makedirs(run_dir, exist_ok=True)
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", a.port), functools.partial(Handler, run_dir=run_dir, page=PAGE_POLICY if a.kind == "policy" else PAGE))
    print(f"dashboard on http://localhost:{a.port}  (run dir {run_dir})", flush=True)
    srv.serve_forever()
