import { Field } from './field';
import { Robot } from './robot';
import './style.css'

export let canvas: HTMLCanvasElement;
export let ctx: CanvasRenderingContext2D;

document.querySelector<HTMLDivElement>('#app')!.innerHTML = /*html*/`
<section class = "bg-gray-600 ">
  <div class="h-screen flex flex-col items-center justify-start p-4">
    <h1 class="text-4xl text-indigo-200 m-4">LIDAR Simulation</h1>
    <canvas id="canvas" class="bg-white border-4 border-indigo-500 rounded-lg shadow-xl h-160 w-160"></canvas>
  </div>
</section>
`

export function initCanvas() {
  canvas = document.getElementById('canvas') as HTMLCanvasElement;
  ctx = canvas.getContext('2d') as CanvasRenderingContext2D;

  const cssWidth = canvas.clientWidth;
  const cssHeight = canvas.clientHeight;

  canvas.width = cssWidth;
  canvas.height = cssHeight;

  ctx.translate(320, 320); 

  ctx.scale(1, -1);  
}

export function render(t: number){
  ctx.clearRect(-320, -320, 640, 640);

  r.render(ctx);
  f.render(ctx);

  window.requestAnimationFrame(render);
}

let f = new Field();
let r = new Robot(0, 0, f.get_obstacle_array(), f.get_obstacle_filter());
initCanvas();

render(0);
