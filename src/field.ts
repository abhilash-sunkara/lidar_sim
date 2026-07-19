import type { point_vector } from "./raycast_utils";

export interface rect_obstacle{
    min: point_vector,
    max: point_vector,
}

export class Field{

    // obstacle_array is used for doing collisions with the lidar rays and is basically a list of rectangles
    // obstacle_filter is used for the mcl algorithm and is 2d grid where each "cell" is the probabilty of an obstacle being there
    // 1 is a known obstacle and 0 is known empty space
    private obstacle_array: rect_obstacle[];
    private obstacle_filter: number[][];

    private keys = {
        p: false,
    };

    private printed = false;

    constructor(){
        this.setupInputListeners();
        this.obstacle_array = [];
        this.add_obstacle(-20, -20, 20, 20);

        this.add_obstacle(-106.667 - 20, -213.333 - 20, -106.667 + 20, -213.333 + 20);
        this.add_obstacle(-213.333 - 20, -106.667 - 20, -213.333 + 20, -106.667 + 20);

        this.add_obstacle(-106.667 - 20, 213.333 - 20, -106.667 + 20, 213.333 + 20);
        this.add_obstacle(-213.333 - 20, 106.667 - 20, -213.333 + 20, 106.667 + 20);

        this.add_obstacle(106.667 - 20, -213.333 - 20, 106.667 + 20, -213.333 + 20);
        this.add_obstacle(213.333 - 20, -106.667 - 20, 213.333 + 20, -106.667 + 20);

        this.add_obstacle(106.667 - 20, 213.333 - 20, 106.667 + 20, 213.333 + 20);
        this.add_obstacle(213.333 - 20, 106.667 - 20, 213.333 + 20, 106.667 + 20);
        //this.add_obstacle(-160, 160, -80, 240);

        this.obstacle_filter = [];
        this.create_obstacle_filter();
        this.obstacle_filter = this.blur_obstacle_filter(this.obstacle_filter);
    }

    add_obstacle(x1: number, y1: number, x2: number, y2: number){
        this.obstacle_array.push({min: {x: x1, y: y1}, max: {x: x2, y: y2}});
    }

    get_obstacle_array(){
        return this.obstacle_array;
    }

    get_obstacle_filter(){
        return this.obstacle_filter;
    }

    private create_obstacle_filter(){
        for(let y = -320; y < 320; y += 10){
            let row = [];
            for(let x = -320; x < 320; x += 10){
                let isObstacle = this.obstacle_array.some((item) => {
                    return (x >= item.min.x && x <= item.max.x && y >= item.min.y && y <= item.max.y);
                })

                //wall check
                isObstacle = isObstacle || (y == -320 || y == 310 || x == -320 || x == -310);
                row.push(isObstacle ? 1 : 0);
            }
            this.obstacle_filter.push(row);
        }
    }

    // blurs obstacle filter to allow/account for some randomness and leniency with hit detection
    private blur_obstacle_filter(grid: number[][]){
        const rows = grid.length;
        const cols = grid[0].length;

        let blurred_array = Array(rows).fill(0).map(() => Array(cols).fill(0));

        const max_radius = 3;

        for(let y = 0; y < rows; y++){
            for(let x = 0; x < cols; x++){
                if(grid[y][x] == 1){
                    blurred_array[y][x] = 1.0;
                    continue;
                }

                // basically draws a circle and finds how close the nearest obstacle is
                // closer obstacle means higher final weight
                let highest_score = 0;
                for(let dy = -max_radius; dy <= max_radius; dy++){
                    for(let dx = -max_radius; dx <= max_radius; dx++){
                        let ny = y + dy;
                        let nx = x + dx;

                        if (ny >= 0 && ny < rows && nx >= 0 && nx < cols) {
                            if(grid[ny][nx] == 1){
                                let distance = Math.sqrt(dx ** 2 + dy ** 2);

                                if (distance <= max_radius) {
                                    let score = 1.0 - (distance / (max_radius + 1));
                                    
                                    if (score > highest_score) {
                                        highest_score = score;
                                    }
                                }
                            }
                        }
                    }
                }
                blurred_array[y][x] = highest_score ** 3; // cubed to make it not linear, makes error/hit detection for mcl algorithm better
            }
        }
        return blurred_array;
    }

    private setupInputListeners() {
        document.addEventListener('keydown', (event) => {
            switch(event.code) {
                case 'KeyP': this.keys.p = true; break;
            }
        });

        document.addEventListener('keyup', (event) => {
            switch(event.code) {
                case 'KeyP': this.keys.p = false; 
                             this.printed = false;
                             break;
            }
        });
    }

    private print_high_weights(grid: number[][]) {
        const TILE_SIZE = 10; 

        for (let y = 0; y < grid.length; y++) {
            for (let x = 0; x < grid[y].length; x++) {
                let weight = grid[y][x];
                
                if (weight > 0.5) {
                    let physicalX = (x - 32) * TILE_SIZE;
                    let physicalY = (y - 32) * TILE_SIZE;
                    
                    console.log(`X: ${physicalX}, Y: ${physicalY} | Weight: ${weight.toFixed(2)}`);
                }
            }
        }
    }

    render(ctx: CanvasRenderingContext2D){
        if(this.keys.p && !this.printed) {
            //this.print_high_weights(this.obstacle_filter)
            this.printed = true;
        };
        ctx.fillStyle = 'red'; 
        this.obstacle_array.forEach((item) => {
            ctx.fillRect(item.min.x, item.min.y, item.max.x - item.min.x, item.max.y - item.min.y);
        })
    }
}