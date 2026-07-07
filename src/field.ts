import type { point_vector } from "./raycast_utils";

export interface rect_obstacle{
    min: point_vector,
    max: point_vector,
}

export class Field{
    private obstacle_array: rect_obstacle[];

    constructor(){
        this.obstacle_array = [];
        this.add_obstacle(160, 160, 240, 240);
        this.add_obstacle(-160, 160, -80, 240);
    }

    add_obstacle(x1: number, y1: number, x2: number, y2: number){
        this.obstacle_array.push({min: {x: x1, y: y1}, max: {x: x2, y: y2}});
    }

    get_obstacle_array(){
        return this.obstacle_array;
    }

    render(ctx: CanvasRenderingContext2D){
        ctx.fillStyle = 'red'; 
        this.obstacle_array.forEach((item) => {
            ctx.fillRect(item.min.x, item.min.y, item.max.x - item.min.x, item.max.y - item.min.y);
        })
    }
}