import type { rect_obstacle } from "./field";
import { check_rect_collision, check_wall_collision, get_distance, type lidar_ray, type point_vector } from "./raycast_utils";

export class Robot{
    private position: point_vector;
    private size: number;
    private speed: number;
    private lidar_array: lidar_ray[];
    private obstacles: rect_obstacle[];
    private lidar_radius: number;

    private keys = {
        w: false,
        a: false,
        s: false,
        d: false,
        p: false,
    };
    
    constructor(start_x: number, start_y: number, obs: rect_obstacle[]) {
        this.position = {x: start_x, y: start_y};
        this.size = 80;
        this.speed = 5;
        this.setupInputListeners();
        this.lidar_array = [];
        this.lidar_radius = 200;
        this.addLidarRays(this.lidar_radius, 100);
        this.obstacles = obs;
    }

    private addLidarRays(radius: number, num_rays: number){
        for(let i = 0; i < num_rays; i++){
            let angle = i * (2 * Math.PI / num_rays);
            let eX = radius * Math.cos(angle) + this.position.x;
            let eY = radius * Math.sin(angle) + this.position.y;
            this.lidar_array.push({start_pos: {x: this.position.x, y: this.position.y}, end_pos: {x: eX, y: eY}, radius: radius, angle: angle})
        }
    }

    private setupInputListeners() {
        document.addEventListener('keydown', (event) => {
            switch(event.code) {
                case 'KeyW': this.keys.w = true; break;
                case 'KeyA': this.keys.a = true; break;
                case 'KeyS': this.keys.s = true; break;
                case 'KeyD': this.keys.d = true; break;
                case 'KeyP': this.keys.p = true; break;
            }
        });

        document.addEventListener('keyup', (event) => {
            switch(event.code) {
                case 'KeyW': this.keys.w = false; break;
                case 'KeyA': this.keys.a = false; break;
                case 'KeyS': this.keys.s = false; break;
                case 'KeyD': this.keys.d = false; break;
                case 'KeyP': this.keys.p = false; break;
            }
        });
    }

    private updateRobotPosition() {
        if (this.keys.w) this.position.y += this.speed;
        if (this.keys.s) this.position.y -= this.speed;
        if (this.keys.a) this.position.x -= this.speed;
        if (this.keys.d) this.position.x += this.speed;
    }

    private updateLidarPosition() {
        this.lidar_array.forEach((item) => {
            let dx = this.position.x - item.start_pos.x;
            let dy = this.position.y - item.start_pos.y;

            item.start_pos.x = this.position.x;
            item.start_pos.y = this.position.y;
            item.end_pos.x = item.end_pos.x + dx;
            item.end_pos.y = item.end_pos.y + dy;

            if(item.radius < this.lidar_radius || Number.isNaN(item.radius)){
                item.end_pos.x = this.lidar_radius * Math.cos(item.angle) + item.start_pos.x;
                item.end_pos.y = this.lidar_radius * Math.sin(item.angle) + item.start_pos.y;
            }


            let new_endpoint: point_vector | null = check_wall_collision({x: item.start_pos.x, y: item.start_pos.y}, {x: item.end_pos.x, y: item.end_pos.y}, item.angle);
            if(new_endpoint){
                item.end_pos.x = new_endpoint.x;
                item.end_pos.y = new_endpoint.y;
            }
            
            this.obstacles.forEach((item_o) => {
                let new_wall_point = check_rect_collision({x: item.start_pos.x, y: item.start_pos.y}, item.angle, this.lidar_radius, item_o);
                if(new_wall_point){
                    item.end_pos.x = new_wall_point.x;
                    item.end_pos.y = new_wall_point.y;
                }
            })
            
            item.radius = get_distance({x: item.start_pos.x, y: item.start_pos.y}, {x: item.end_pos.x, y: item.end_pos.y})
        })
    }

    private printLidarMap(){
        this.lidar_array.forEach((item) => {
            //console.log("start x: " + item.start_pos.x + ", start y: " + item.start_pos.y + " end x: " + item.end_pos.x + ", end y: " + item.end_pos.y)
            //console.log("Radius: " + item.radius + ", Angle: " + item.angle)
        })
    }

    render(ctx: CanvasRenderingContext2D){
        if(this.keys.p) {this.printLidarMap()};
        this.updateRobotPosition();
        this.updateLidarPosition();
        ctx.fillStyle = 'black'; 
        
        

        ctx.beginPath();

        this.lidar_array.forEach((item) => {
            ctx.moveTo(item.start_pos.x, item.start_pos.y);     
            ctx.lineTo(item.end_pos.x, item.end_pos.y);
        })

        ctx.strokeStyle = 'blue';
        ctx.lineWidth = 2;
        ctx.stroke();

        ctx.fillRect(this.position.x - this.size / 2, this.position.y - this.size / 2, this.size, this.size);
    }
};