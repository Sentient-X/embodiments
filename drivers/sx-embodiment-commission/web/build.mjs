// Bundle installed workspace vendor dependencies; deployment has no CDN or Node dependency.
import {createRequire} from 'node:module';
import {resolve,dirname} from 'node:path';
import {fileURLToPath} from 'node:url';
import {cpSync,mkdirSync,readdirSync} from 'node:fs';
const here=dirname(fileURLToPath(import.meta.url));const root=resolve(here,'../../../../..');
const require=createRequire(import.meta.url);
const esbuildPath=readdirSync(resolve(root,'node_modules/.pnpm')).filter(n=>n.startsWith('esbuild@')).sort().at(-1);
const {build}=require(resolve(root,'node_modules/.pnpm',esbuildPath,'node_modules/esbuild'));
const out=resolve(process.argv[2]||resolve(here,'dist'));mkdirSync(out,{recursive:true});
await build({entryPoints:[resolve(here,'app.js')],bundle:true,minify:true,format:'esm',outfile:resolve(out,'app.js'),nodePaths:[resolve(root,'platform/console/node_modules')],target:'es2022'});
for(const name of ['index.html','style.css'])cpSync(resolve(here,name),resolve(out,name));
mkdirSync(resolve(out,'model'),{recursive:true});
for(const name of ['robot.urdf','pack.glb'])cpSync(resolve(root,'packages/sx-embodiments/render_packs/sentient-static-upper-body',name),resolve(out,'model',name));
console.log(out);
