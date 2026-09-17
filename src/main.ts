import { Field } from './field';
import { OpponentRobot } from './opponent_robot';
import { Robot } from './robot';
import './style.css'

export let canvas: HTMLCanvasElement;
export let ctx: CanvasRenderingContext2D;

document.querySelector<HTMLDivElement>('#app')!.innerHTML = /*html*/`
<section class = "bg-gray-600 ">
  <div class="h-screen flex flex-col items-center justify-start p-4">
    <h1 class="text-4xl text-indigo-200 m-4">LIDAR Simulation</h1>
    <div class = "w-full flex justify-evenly items-start">
      <canvas id="canvas" class="bg-white border-4 border-indigo-500 rounded-lg shadow-xl h-160 w-160"></canvas>
      <div class = "flex-col items-start justify-center">
        <h1 class = "text-3xl text-indigo-200"> 
          Actual Position
        </h1>
        <div class = "flex flex-row">
          <h1 id = "actual-x" class = "text-3xl text-indigo-200 mr-2"/> 
          <h1 id = "actual-y" class = "text-3xl text-indigo-200 mr-2"/> 
          <h1 id = "actual-heading" class = "text-3xl text-indigo-200"/> 
        </div>
        <h1 class = "text-3xl text-indigo-200"> 
          Expected Position
        </h1>
        <div class = "flex flex-row">
          <h1 id = "expected-x" class = "text-3xl text-indigo-200 mr-2"/> 
          <h1 id = "expected-y" class = "text-3xl text-indigo-200 mr-2"/> 
          <h1 id = "expected-heading" class = "text-3xl text-indigo-200"/> 
        </div>
        <div class="flex flex-col w-full">
          <label htmlFor="startingAngle" class="text-indigo-200 text-lg mb-1">Starting Angle</label>
          <input 
            type="number"
            id="startingAngle"
            name="startingAngle" 
            value="0"
            step="any"
            class="border p-2 rounded text-black bg-white"
          />
        </div>
        <div class="flex flex-col w-full">
          <label htmlFor="endingAngle" class="text-indigo-200 text-lg mb-1">Ending Angle</label>
          <input 
            type="number"
            id="endingAngle"
            name="endingAngle" 
            value="6.28"
            step="any"
            class="border p-2 rounded text-black bg-white"
          />
        </div>
        <div class="flex flex-col w-full">
          <label htmlFor="numRays" class="text-indigo-200 text-lg mb-1"># Rays</label>
          <input 
            type="number"
            id="numRays"
            name="numRays" 
            value="100"
            step="any"
            class="border p-2 rounded text-black bg-white"
          />
        </div>
        <div class="flex flex-col w-full">
          <label htmlFor="lidarRadius" class="text-indigo-200 text-lg mb-1">Lidar Radius</label>
          <input 
            type="number"
            id="lidarRadius"
            name="lidarRadius" 
            value="300"
            step="any"
            class="border p-2 rounded text-black bg-white"
          />
        </div>
    </div>
    
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

const startAngleInput = document.getElementById('startingAngle') as HTMLInputElement;
const endAngleInput = document.getElementById('endingAngle') as HTMLInputElement;
const numRaysInput = document.getElementById('numRays') as HTMLInputElement;
const lidarRadiusInput = document.getElementById('lidarRadius') as HTMLInputElement;

export function render(){
  ctx.clearRect(-320, -320, 640, 640);

  const startAngle = parseFloat(startAngleInput.value) || 0;
  const endAngle = parseFloat(endAngleInput.value) || 0;
  const numRays = parseFloat(numRaysInput.value) || 0;
  const lidarRadius = parseFloat(lidarRadiusInput.value) || 0;

  r.render(ctx, startAngle, endAngle, numRays, lidarRadius);
  f.render(ctx);
  or.render(ctx);

  window.requestAnimationFrame(render);
}

let f = new Field();
let or = new OpponentRobot();
let r = new Robot(-240, 0, f.get_obstacle_array(), f.get_obstacle_filter(), or.get_pos_object());

initCanvas();

render();
