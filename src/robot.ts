import type {rect_obstacle} from "./field";
import { check_rect_collision, check_wall_collision, get_distance, type lidar_ray, type point_vector } from "./raycast_utils";

interface mcl_point{
    position: point_vector,
    weight: number,
}

export class Robot{
    private position: point_vector;
    private size: number;
    private speed: number;
    private lidar_array: lidar_ray[];
    private obstacles: rect_obstacle[];
    private lidar_radius: number;

    private field_map: number[][];

    private mcl_points: mcl_point[];
    private mcl_displacement: point_vector = {x: 0, y: 0};

    private opp_rob_pos: point_vector;

    private printed = false;

    private keys = {
        w: false,
        a: false,
        s: false,
        d: false,
        p: false,
    };
    
    constructor(start_x: number, start_y: number, obs: rect_obstacle[], obs_filter: number[][], orp: point_vector) {
        this.position = {x: start_x, y: start_y};
        this.size = 80;
        this.speed = 5;
        this.setupInputListeners();
        this.lidar_array = [];
        this.lidar_radius = 300;
        this.addLidarRays(this.lidar_radius, 100);
        this.obstacles = obs;

        this.mcl_points = [];
        this.addMCLPoints(10);

        this.field_map = obs_filter;

        this.opp_rob_pos = orp;
    }

    private addMCLPoints(resolution: number){
        for(let x = -320; x < 320; x += resolution){
            for(let y = -320; y < 320; y += resolution){
                this.mcl_points.push({position: {x: x, y: y}, weight: 0.0});
            }
        }
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
                case 'KeyP': this.keys.p = false;
                             this.printed = false; 
                             break;
            }
        });
    }

    private updateRobotPosition() {
        if (this.keys.w) {
            this.position.y += this.speed;
            this.mcl_displacement.y += this.speed;
        }
        if (this.keys.s) {
            this.position.y -= this.speed;
            this.mcl_displacement.y -= this.speed;
        }
        if (this.keys.a) {
            this.position.x -= this.speed;
            this.mcl_displacement.x -= this.speed;
        }
        if (this.keys.d) {
            this.position.x += this.speed;
            this.mcl_displacement.x += this.speed;
        }
    }

    //Box-Muller transform converts uniform noise into gaussian noise
    private getGaussianNoise(mean = 0, stdDev = 1) {
        let u1 = 1 - Math.random(); 
        let u2 = 1 - Math.random(); 
        let z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
        return z0 * stdDev + mean;
    }

    private updateLidarPosition() {
        this.lidar_array.forEach((item) => {
            //let dx = this.position.x - item.start_pos.x;
            //let dy = this.position.y - item.start_pos.y;

            item.start_pos.x = this.position.x;
            item.start_pos.y = this.position.y;
            //item.end_pos.x = item.end_pos.x + dx;
            //item.end_pos.y = item.end_pos.y + dy;

            item.end_pos.x = this.lidar_radius * Math.cos(item.angle) + item.start_pos.x;
            item.end_pos.y = this.lidar_radius * Math.sin(item.angle) + item.start_pos.y;

            

            let new_endpoint: point_vector | null = check_wall_collision({x: item.start_pos.x, y: item.start_pos.y}, {x: item.end_pos.x, y: item.end_pos.y}, item.angle);
            if(new_endpoint){
                item.end_pos.x = new_endpoint.x;
                item.end_pos.y = new_endpoint.y;
            }

            let new_wall_point_real: point_vector | null = {x: 0, y:0};
            this.obstacles.forEach((item_o) => {
                let new_wall_point = check_rect_collision({x: item.start_pos.x, y: item.start_pos.y}, item.angle, this.lidar_radius, item_o);
                if(new_wall_point){
                    item.end_pos.x = new_wall_point.x;
                    item.end_pos.y = new_wall_point.y;
                    new_wall_point_real = new_wall_point;
                }
            })

            let new_orobot_point = check_rect_collision({x: item.start_pos.x, y: item.start_pos.y}, item.angle, this.lidar_radius, {min: {x: this.opp_rob_pos.x - 40, y: this.opp_rob_pos.y - 40}, max: {x: this.opp_rob_pos.x + 40, y: this.opp_rob_pos.y + 40}});
            if(new_orobot_point){
                item.end_pos.x = new_orobot_point.x;
                item.end_pos.y = new_orobot_point.y;
            }
            console.log(new_wall_point_real)
            if(new_orobot_point && new_wall_point_real){
                if(get_distance({x: item.start_pos.x, y: item.start_pos.y}, {x: new_orobot_point.x, y: new_orobot_point.y}) < get_distance({x: item.start_pos.x, y: item.start_pos.y}, {x: new_wall_point_real.x, y: new_wall_point_real.y})){
                    item.end_pos.x = new_orobot_point.x;
                    item.end_pos.y = new_orobot_point.y;
                } else if (new_wall_point_real.x != 0 && new_wall_point_real.y != 0) {
                    item.end_pos.x = new_wall_point_real.x;
                    item.end_pos.y = new_wall_point_real.y;
                }
            }
            //converts to polar
            let dx = item.end_pos.x - item.start_pos.x; 
            let dy = item.end_pos.y - item.start_pos.y;
            let distance = Math.sqrt(dx * dx + dy * dy);
            let angle = Math.atan2(dy, dx);

            //applies noise to distance
            let range_noise = this.getGaussianNoise(0, 2.0); 
            let noisy_distance = distance + range_noise;

            //converts to rectangular
            item.end_pos.x = noisy_distance * Math.cos(angle) + item.start_pos.x;
            item.end_pos.y = noisy_distance * Math.sin(angle) + item.start_pos.y;
            
            item.radius = get_distance({x: item.start_pos.x, y: item.start_pos.y}, {x: item.end_pos.x, y: item.end_pos.y})
        })
    }

    private analyzeLidarPoints() {
        // Loops through all lidar rays, grabs their "relative" distance and tags them with if it hit a wall or not

        let processed_rays = this.lidar_array.map((item) => {
            let rel_x = item.end_pos.x - item.start_pos.x;
            let rel_y = item.end_pos.y - item.start_pos.y;
            //console.log(item.radius);
            
            return {
                x: rel_x, 
                y: rel_y, 
                is_hit: item.radius < (this.lidar_radius - 1)
            };
        });

        // Loops through each particle in the sim, "moves"/projevcts the processed rays to the position of each point
        // After this compares the actual "is_hit" value with the expected one if the robot was at the position of the mcl point
        this.mcl_points.forEach((particle) => {
            let total_error = 0; 
            
            let real_position: point_vector = {
                x: particle.position.x + this.mcl_displacement.x, 
                y: particle.position.y + this.mcl_displacement.y
            };

            let print = real_position.x == 0 && real_position.y == 0;
            
            processed_rays.forEach((ray) => {
                let proj_x = ray.x + real_position.x;
                let proj_y = ray.y + real_position.y;
                
                if(proj_x > -320 && proj_x < 320 && proj_y > -320 && proj_y < 320) {
                    let gridX = Math.floor(proj_x / 10) + 32;
                    let gridY = Math.floor(proj_y / 10) + 32;
                    
                    let map_value = this.field_map[gridY][gridX]; 
                    /* if(print){
                        console.log("Map value: " + map_value);
                    } */
                    if (ray.is_hit) {
                        total_error += (1.0 - map_value);
                        
                        if(print){
                            //console.log("Expected a ray hit here, map value is " + map_value);
                        }
                    } else {
                        total_error += (map_value * 2.0); 

                        if(print){
                            //console.log("Expected empty space, map value is " + map_value);
                        }
                    }
                } else {
                    if (ray.is_hit) {
                        total_error += 1.0; 
                    } else {
                        total_error += 0; 
                    }
                }
            });
            
            particle.weight = 1000.0 / (total_error + 1.0);
            if(print){
                console.log("Final error is " + particle.weight)
            } 
        });

        
        this.mcl_points.sort((a, b) => (b.weight - a.weight));
    }

    private getEstimatedPosition(): point_vector {
    
        this.mcl_points.sort((a, b) => b.weight - a.weight);

        
        let top_particles = this.mcl_points.slice(0, 5);

        let total_weight = 0;
        let weighted_x = 0;
        let weighted_y = 0;

        
        top_particles.forEach(p => {
            total_weight += p.weight;
            weighted_x += (p.position.x + this.mcl_displacement.x) * p.weight;
            weighted_y += (p.position.y + this.mcl_displacement.y) * p.weight;
        });

        
        return {
            x: weighted_x / total_weight,
            y: weighted_y / total_weight
        };
    }

    private printLidarMap(){
        this.lidar_array.forEach((item) => {
            //console.log("start x: " + item.start_pos.x + ", start y: " + item.start_pos.y + " end x: " + item.end_pos.x + ", end y: " + item.end_pos.y)
            //console.log("Radius: " + item.radius + ", Angle: " + item.angle)
        })

        this.analyzeLidarPoints();
        this.mcl_points.forEach((item) => {
            
            if(item.weight > 0){
                //console.log(`X: ${item.position.x + this.mcl_displacement.x}, Y: ${item.position.y + this.mcl_displacement.y}, Weight: ${item.weight}`);
                this.printed = true;
            }
        })

        let pos = this.getEstimatedPosition();
        console.log("estimated X: " + pos.x + this.mcl_displacement.x + ", estimated Y: " + pos.y + this.mcl_displacement.y);

        this.printed = true;
    }

    

    render(ctx: CanvasRenderingContext2D){
        if(this.keys.p && !this.printed) {this.printLidarMap()};
        this.updateRobotPosition();
        this.updateLidarPosition();
        //this.analyzeLidarPoints();
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