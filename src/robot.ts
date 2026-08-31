import type {rect_obstacle} from "./field";
import { check_rect_collision, check_wall_collision, get_distance, type lidar_ray, type point_vector } from "./raycast_utils";

interface mcl_point{
    position: point_vector,
    weight: number,
}

interface cluster{
    points: lidar_ray[],
    obstacle_type: string,
    min: point_vector,
    max: point_vector,

}

const ObstacleType = {
    WALL: "WALL",
    ROBOT : "ROBOT",
    TEMP : "TEMP",
    FIELD_OBSTACLE : "FIELD_OBSTACLE"
}

export class Robot{
    private position: point_vector;
    private size: number;
    private speed: number;
    private lidar_array: lidar_ray[];
    private obstacles: rect_obstacle[];
    private lidar_radius: number;

    private detected_objects: cluster[] = [];

    private field_map: number[][];

    private mcl_points: mcl_point[];

    private opp_rob_pos: point_vector;

    private printed = false;

    private expected_position: point_vector = {x: 0, y: 0};

    private starting_angle: number = 0 * Math.PI;
    private ending_angle: number = 1 * Math.PI;

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
        this.addMCLPoints(300);

        this.field_map = obs_filter;

        this.opp_rob_pos = orp;
    }

    private addMCLPoints(num_particles: number) {
        this.mcl_points = [];
        for (let i = 0; i < num_particles; i++) {
            let rx = (Math.random() * 640) - 320;
            let ry = (Math.random() * 640) - 320;
            this.mcl_points.push({ position: { x: rx, y: ry }, weight: 1.0 / num_particles });
        }
    }

    private addLidarRays(radius: number, num_rays: number){
        this.lidar_array = [];
        for(let i = 0; i < num_rays; i++){
            let angle = i * (2 * Math.PI / num_rays);
            let eX = radius * Math.cos(angle) + this.position.x;
            let eY = radius * Math.sin(angle) + this.position.y;

            if(angle >= this.starting_angle && angle <= this.ending_angle){
                this.lidar_array.push({start_pos: {x: this.position.x, y: this.position.y}, end_pos: {x: eX, y: eY}, radius: radius, angle: angle})
            }
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
        let dx = 0;
        let dy = 0;

        if (this.keys.w) dy += this.speed;
        if (this.keys.s) dy -= this.speed;
        if (this.keys.a) dx -= this.speed;
        if (this.keys.d) dx += this.speed;

        if (dx !== 0 || dy !== 0) {
            this.position.x += dx;
            this.position.y += dy;

            //odom noise
            this.mcl_points.forEach((p) => {
                p.position.x += dx + this.getGaussianNoise(0, 1.5);
                p.position.y += dy + this.getGaussianNoise(0, 1.5);
            });
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
            //console.log(new_wall_point_real)
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

        let sum_weights = 0;
        // Loops through each particle in the sim, "moves"/projevcts the processed rays to the position of each point
        // After this compares the actual "is_hit" value with the expected one if the robot was at the position of the mcl point
        this.mcl_points.forEach((particle) => {
            let total_error = 0; 
            
            let real_position = particle.position;

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
            sum_weights += particle.weight;
            if(print){
                console.log("Final error is " + particle.weight)
            } 
        });

        this.mcl_points.forEach((particle) => {
            particle.weight /= sum_weights;
        });

        
       // this.mcl_points.sort((a, b) => (b.weight - a.weight));
    }

    private getEstimatedPosition(): point_vector {
    
        this.mcl_points.sort((a, b) => b.weight - a.weight);

        
        let top_particles = this.mcl_points.slice(0, 5);

        let total_weight = 0;
        let weighted_x = 0;
        let weighted_y = 0;

        
        top_particles.forEach(p => {
            total_weight += p.weight;
            weighted_x += p.position.x * p.weight; 
            weighted_y += p.position.y * p.weight;
        });

        
        return {
            x: weighted_x / total_weight,
            y: weighted_y / total_weight
        };
    }

    //stochastic universal sampling with random injection cause point cloud would get stuck in some location
    private resampleParticles() {
        let new_particles: mcl_point[] = [];
        let num_particles = this.mcl_points.length;
        
        let random_injection_rate = 0.05; 
        let num_resample = Math.floor(num_particles * (1.0 - random_injection_rate));
        let num_random = num_particles - num_resample;

        //interval math
        let r = Math.random() / num_particles; 
        let c = this.mcl_points[0].weight;
        let index = 0;

        for (let i = 0; i < num_resample; i++) {
            let u = r + (i / num_particles);
            
            while (u > c) {
                index = (index + 1) % num_particles;
                c += this.mcl_points[index].weight;
            }
            
            new_particles.push({
                position: { x: this.mcl_points[index].position.x, y: this.mcl_points[index].position.y },
                weight: 1.0 / num_particles 
            });
        }

        //random injection to allow points to escape out of "local minimum"
        for (let i = 0; i < num_random; i++) {
            let rx = (Math.random() * 640) - 320;
            let ry = (Math.random() * 640) - 320;
            new_particles.push({ 
                position: { x: rx, y: ry }, 
                weight: 1.0 / num_particles 
            });
        }

        this.mcl_points = new_particles;
    }

    private generateClusters() {
        this.detected_objects = [];
        if (!this.lidar_array || this.lidar_array.length === 0) return;

        let current_cluster: cluster = { points: [], obstacle_type: ObstacleType.TEMP, min: {x: 0, y: 0}, max: {x: 0, y: 0} };
        let prev_point: lidar_ray = this.lidar_array[0];
        
        current_cluster.points.push(prev_point);

        for (let i = 1; i < this.lidar_array.length; i++) {
            let item = this.lidar_array[i];
            let distance_between = get_distance(item.end_pos, prev_point.end_pos);
            
            let average_radius = (item.radius + prev_point.radius) / 2;

            let expected_gap = average_radius * 0.0628;
            let threshold = (expected_gap * 1.5) + 10; 

            if (distance_between < threshold) {
                current_cluster.points.push(item);
            } else {
                if (current_cluster.points.length > 1) {
                    let sum_radius = 0;
                    for (let p of current_cluster.points) {
                        sum_radius += p.radius;
                    }
                    let avg_radius = sum_radius / current_cluster.points.length;

                    if (avg_radius < this.lidar_radius - 20) {
                        this.detected_objects.push(current_cluster);
                    }
                }
                current_cluster = { points: [item], obstacle_type: ObstacleType.TEMP,  min: {x: 0, y: 0}, max: {x: 0, y: 0}};
            }
            
            prev_point = item;
        }

        if (current_cluster.points.length > 1) {
            this.detected_objects.push(current_cluster);
        }

        if (this.detected_objects.length > 1) {
            let first_cluster = this.detected_objects[0];
            let last_cluster = this.detected_objects[this.detected_objects.length - 1];

            let first_point = first_cluster.points[0];
            let last_point = last_cluster.points[last_cluster.points.length - 1];

            let wrap_distance = get_distance(first_point.end_pos, last_point.end_pos);
            let average_wrap_radius = (first_point.radius + last_point.radius) / 2;
            let wrap_threshold = (average_wrap_radius * 0.0628 * 1.5) + 10;

            if (wrap_distance < wrap_threshold) {
                first_cluster.points = last_cluster.points.concat(first_cluster.points);
                this.detected_objects.pop();
            }
        }

    }

    private analyzeClusters(){
        this.detected_objects.forEach((item) => {
            let minX = 320;
            let maxX = -320;
            let minY = 320;
            let maxY = -320;

            item.points.forEach((point) => {
                if(point.end_pos.x < minX){
                    minX = point.end_pos.x;
                }
                if(point.end_pos.x > maxX){
                    maxX = point.end_pos.x;
                }
                if(point.end_pos.y < minY){
                    minY = point.end_pos.y;
                }
                if(point.end_pos.y > maxY){
                    maxY = point.end_pos.y;
                }
            })

            if((maxX - minX > 150) || (maxY - minY > 150)){
                item.obstacle_type = ObstacleType.WALL
            } else {
                item.obstacle_type = ObstacleType.FIELD_OBSTACLE
            }
            item.min = {x: minX, y: minY}
            item.max = {x: maxX, y: maxY}
        })
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

        //let pos = this.getEstimatedPosition();
        //console.log("estimated X: " + pos.x + this.mcl_displacement.x + ", estimated Y: " + pos.y + this.mcl_displacement.y);
        //this.generateClusters();
        //this.analyzeClusters();
        this.detected_objects.forEach((item) => {console.log(item)})
        this.printed = true;
    }

    

    render(ctx: CanvasRenderingContext2D, s_angle: number, e_angle: number){
        if(this.keys.p && !this.printed) {this.printLidarMap()};
        this.updateRobotPosition();
        this.updateLidarPosition();
        this.generateClusters();
        this.analyzeClusters();

        this.analyzeLidarPoints();
        this.expected_position = this.getEstimatedPosition();
        this.resampleParticles();


        if(s_angle != this.starting_angle || e_angle != this.ending_angle){
            this.starting_angle = s_angle;
            this.ending_angle = e_angle;
            console.log("trying to add new rays");
            this.addLidarRays(this.lidar_radius, 100);
        }

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

        ctx.beginPath(); 

        this.detected_objects.forEach((item) => {
            ctx.rect(item.min.x, item.min.y, item.max.x - item.min.x, item.max.y - item.min.y);
        })

        ctx.strokeStyle = 'purple'; 
        ctx.lineWidth = 3;
        ctx.stroke();

        ctx.fillRect(this.position.x - this.size / 2, this.position.y - this.size / 2, this.size, this.size);

        ctx.fillStyle = 'orange';
        this.mcl_points.forEach(p => {
            ctx.fillRect(p.position.x, p.position.y, 2, 2);
        });

        const actual_x_element = document.getElementById("actual-x");
        if(actual_x_element){
            actual_x_element.innerText = `${this.position.x},`
        }

        const actual_y_element = document.getElementById("actual-y");
        if(actual_y_element){
            actual_y_element.innerText = `${this.position.y}`
        }

        const expected_x_element = document.getElementById("expected-x");
        if(expected_x_element){
            expected_x_element.innerText = `${this.expected_position.x.toFixed(1)},`
        }

        const expected_y_element = document.getElementById("expected-y");
        if(expected_y_element){
            expected_y_element.innerText = `${this.expected_position.y.toFixed(1)}`
        }
    }
};