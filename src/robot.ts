import type {rect_obstacle} from "./field";
import { check_rect_collision, check_wall_collision, get_distance, type lidar_ray, type point_vector } from "./raycast_utils";

interface mcl_point{
    position: position_t,
    weight: number,
}

interface position_t{
    x: number;
    y: number;
    heading: number;
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
    private position: position_t;
    //private heading: number;
    private size: number;
    private speed: number;
    private lidar_array: lidar_ray[];
    private obstacles: rect_obstacle[];
    private lidar_radius: number;
    //private need_to_update_lidar_angle: boolean = false;

    private detected_objects: cluster[] = [];

    private field_map: number[][];

    private mcl_points: mcl_point[];

    private opp_rob_pos: point_vector;

    private printed = false;

    private expected_position: position_t = {x: 0, y: 0, heading: 0};

    private starting_angle: number = 0 * Math.PI;
    private ending_angle: number = 1 * Math.PI;

    private num_rays: number = 100;

    private keys = {
        w: false,
        a: false,
        s: false,
        d: false,
        q: false,
        e: false,
        p: false,
    };
    
    constructor(start_x: number, start_y: number, obs: rect_obstacle[], obs_filter: number[][], orp: point_vector) {
        this.position = {x: start_x, y: start_y, heading: 0};
        //this.heading = 0;
        this.size = 80;
        this.speed = 5;
        this.setupInputListeners();
        this.lidar_array = [];
        this.lidar_radius = 300;
        this.addLidarRays(this.lidar_radius, this.num_rays);
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
            let heading = 0;//(Math.random() * 360);
            this.mcl_points.push({ position: { x: rx, y: ry, heading}, weight: 1.0 / num_particles });
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
                case 'KeyQ': this.keys.q = true; break;
                case 'KeyE': this.keys.e = true; break;
            }
        });

        document.addEventListener('keyup', (event) => {
            switch(event.code) {
                case 'KeyW': this.keys.w = false; break;
                case 'KeyA': this.keys.a = false; break;
                case 'KeyS': this.keys.s = false; break;
                case 'KeyD': this.keys.d = false; break;
                case 'KeyQ': this.keys.q = false; break;
                case 'KeyE': this.keys.e = false; break;
                case 'KeyP': this.keys.p = false;
                             this.printed = false; 
                             break;
            }
        });
    }

    private updateRobotPosition() {
        let dx = 0;
        let dy = 0;
        let d_theta = 0;

        if (this.keys.w) dy += this.speed;
        if (this.keys.s) dy -= this.speed;
        if (this.keys.a) dx -= this.speed;
        if (this.keys.d) dx += this.speed;
        if (this.keys.q) d_theta -= this.speed/50;
        if (this.keys.e) d_theta += this.speed/50;
        //if(this.keys.e || this.keys.q) this.need_to_update_lidar_angle = true;

        if (dx !== 0 || dy !== 0 || d_theta !== 0) {
            this.position.x += dx;
            this.position.y += dy;
            this.position.heading += d_theta;

            //odom noise
            this.mcl_points.forEach((p) => {
                p.position.x += dx + this.getGaussianNoise(0, 1.5);
                p.position.y += dy + this.getGaussianNoise(0, 1.5);
                p.position.heading += d_theta + this.getGaussianNoise(0, 0.2);
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
        let heading_offset_rad = (this.position.heading / 180) * Math.PI;

        this.lidar_array.forEach((item) => {
            item.start_pos.x = this.position.x;
            item.start_pos.y = this.position.y;

            let global_angle = item.angle + heading_offset_rad;

            item.end_pos.x = this.lidar_radius * Math.cos(global_angle) + item.start_pos.x;
            item.end_pos.y = this.lidar_radius * Math.sin(global_angle) + item.start_pos.y;

            let new_endpoint: point_vector | null = check_wall_collision({x: item.start_pos.x, y: item.start_pos.y}, {x: item.end_pos.x, y: item.end_pos.y}, global_angle);
            if(new_endpoint){
                item.end_pos.x = new_endpoint.x;
                item.end_pos.y = new_endpoint.y;
            }

            let new_wall_point_real: point_vector | null = {x: 0, y:0};
            this.obstacles.forEach((item_o) => {
                let new_wall_point = check_rect_collision({x: item.start_pos.x, y: item.start_pos.y}, global_angle, this.lidar_radius, item_o);
                if(new_wall_point){
                    item.end_pos.x = new_wall_point.x;
                    item.end_pos.y = new_wall_point.y;
                    new_wall_point_real = new_wall_point;
                }
            })

            let new_orobot_point = check_rect_collision({x: item.start_pos.x, y: item.start_pos.y}, global_angle, this.lidar_radius, {min: {x: this.opp_rob_pos.x - 40, y: this.opp_rob_pos.y - 40}, max: {x: this.opp_rob_pos.x + 40, y: this.opp_rob_pos.y + 40}});
            if(new_orobot_point){
                item.end_pos.x = new_orobot_point.x;
                item.end_pos.y = new_orobot_point.y;
            }
            
            if(new_orobot_point && new_wall_point_real){
                if(get_distance({x: item.start_pos.x, y: item.start_pos.y}, {x: new_orobot_point.x, y: new_orobot_point.y}) < get_distance({x: item.start_pos.x, y: item.start_pos.y}, {x: new_wall_point_real.x, y: new_wall_point_real.y})){
                    item.end_pos.x = new_orobot_point.x;
                    item.end_pos.y = new_orobot_point.y;
                } else if (new_wall_point_real.x != 0 && new_wall_point_real.y != 0) {
                    item.end_pos.x = new_wall_point_real.x;
                    item.end_pos.y = new_wall_point_real.y;
                }
            }
            
            let dx = item.end_pos.x - item.start_pos.x; 
            let dy = item.end_pos.y - item.start_pos.y;
            let distance = Math.sqrt(dx * dx + dy * dy);
            let final_noisy_global_angle = Math.atan2(dy, dx);

            let range_noise = this.getGaussianNoise(0, 2.0); 
            let noisy_distance = distance + range_noise;

            item.end_pos.x = noisy_distance * Math.cos(final_noisy_global_angle) + item.start_pos.x;
            item.end_pos.y = noisy_distance * Math.sin(final_noisy_global_angle) + item.start_pos.y;
            
            item.radius = get_distance({x: item.start_pos.x, y: item.start_pos.y}, {x: item.end_pos.x, y: item.end_pos.y});
        })
        
        //this.need_to_update_lidar_angle = false;
    }

    private analyzeLidarPoints() {
        let field_obstacle_rays = new Set<lidar_ray>();
        this.detected_objects.forEach((item) => {
            if(item.obstacle_type == ObstacleType.FIELD_OBSTACLE) {
                item.points.forEach((l_item) => {
                    field_obstacle_rays.add(l_item);
                });
            }
        });

        let processed_rays = this.lidar_array.map((item) => {
            let is_hit = item.radius < (this.lidar_radius - 1);
            let is_dynamic_obstacle = is_hit && !field_obstacle_rays.has(item);

            return {
                distance: item.radius,
                local_angle: item.angle, 
                is_hit: is_hit,
                ignore_in_mcl: is_dynamic_obstacle
            };
        });

        let sum_weights = 0;

        this.mcl_points.forEach((particle) => {
            let total_error = 0; 
            let real_position = particle.position;
            
            let particle_heading_rad = (real_position.heading / 180) * Math.PI;
            
            processed_rays.forEach((ray) => {
                if (ray.ignore_in_mcl) {
                    return; 
                }

                let global_ray_angle = particle_heading_rad + ray.local_angle;

                
                let proj_x = real_position.x + ray.distance * Math.cos(global_ray_angle);
                let proj_y = real_position.y + ray.distance * Math.sin(global_ray_angle);
                
                if(proj_x > -320 && proj_x < 320 && proj_y > -320 && proj_y < 320) {
                    let gridX = Math.floor(proj_x / 10) + 32;
                    let gridY = Math.floor(proj_y / 10) + 32;
                    
                    let map_value = this.field_map[gridY][gridX]; 
                    
                    if (ray.is_hit) {
                        total_error += (1.0 - map_value);
                    } else {
                        total_error += (map_value * 2.0); 
                    }
                } else {
                    if (ray.is_hit) {
                        total_error += 1.0; 
                    }
                }
            });
            
            particle.weight = 1000.0 / (total_error + 1.0);
            sum_weights += particle.weight;
        });

        this.mcl_points.forEach((particle) => {
            particle.weight /= sum_weights;
        });
    }

    private getEstimatedPosition(): position_t {
    
        this.mcl_points.sort((a, b) => b.weight - a.weight);

        
        let top_particles = this.mcl_points.slice(0, 5);

        let total_weight = 0;
        let weighted_x = 0;
        let weighted_y = 0;
        let weighted_heading = 0;

        
        top_particles.forEach(p => {
            total_weight += p.weight;
            weighted_x += p.position.x * p.weight; 
            weighted_y += p.position.y * p.weight;
            weighted_heading += p.position.heading * p.weight;
        });

        
        return {
            x: weighted_x / total_weight,
            y: weighted_y / total_weight,
            heading: weighted_heading / total_weight
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
                position: { x: this.mcl_points[index].position.x, y: this.mcl_points[index].position.y, heading: this.mcl_points[index].position.heading},
                weight: 1.0 / num_particles 
            });
        }

        //random injection to allow points to escape out of "local minimum"
        for (let i = 0; i < num_random; i++) {
            let rx = (Math.random() * 640) - 320;
            let ry = (Math.random() * 640) - 320;
            let r_heading = (Math.random() * 360);
            new_particles.push({ 
                position: { x: rx, y: ry, heading: r_heading}, 
                weight: 1.0 / num_particles 
            });
        }

        this.mcl_points = new_particles;
    }

    private generateClusters() {
        this.detected_objects = [];
        if (!this.lidar_array || this.lidar_array.length === 0) return;

        const DELTA_PHI = 0.0628; // Angular resolution in radians 
        const LAMBDA = 0.174;    // Expected incidence angle 
        const SIGMA_R = 2;     // Standard deviation

        const ADAPTIVE_FACTOR = Math.sin(DELTA_PHI) / Math.sin(LAMBDA - DELTA_PHI);

        let current_cluster: cluster = { 
            points: [], 
            obstacle_type: ObstacleType.TEMP, 
            min: {x: 0, y: 0}, 
            max: {x: 0, y: 0} 
        };
        
        let prev_point: lidar_ray = this.lidar_array[0];
        current_cluster.points.push(prev_point);

        for (let i = 1; i < this.lidar_array.length; i++) {
            let item = this.lidar_array[i];
            let distance_between = get_distance(item.end_pos, prev_point.end_pos);
            let radius_diff = Math.abs(item.radius - prev_point.radius);

            let r_min = Math.min(prev_point.radius, item.radius);
            let threshold = (r_min * ADAPTIVE_FACTOR) + (3 * SIGMA_R);

            if (distance_between < threshold && radius_diff < 20) {
                current_cluster.points.push(item);
            } else {
                this.finalizeCluster(current_cluster);
                current_cluster = { 
                    points: [item], 
                    obstacle_type: ObstacleType.TEMP, 
                    min: {x: 0, y: 0}, 
                    max: {x: 0, y: 0}
                };
            }
            
            prev_point = item;
        }

        // Finalize trailing cluster
        this.finalizeCluster(current_cluster);

        // Handle 360-degree array wrap-around
        if (this.detected_objects.length > 1) {
            let first_cluster = this.detected_objects[0];
            let last_cluster = this.detected_objects[this.detected_objects.length - 1];

            let first_point = first_cluster.points[0];
            let last_point = last_cluster.points[last_cluster.points.length - 1];

            let wrap_distance = get_distance(first_point.end_pos, last_point.end_pos);
            let r_min_wrap = Math.min(first_point.radius, last_point.radius);
            let wrap_threshold = (r_min_wrap * ADAPTIVE_FACTOR) + (3 * SIGMA_R);

            if (wrap_distance < wrap_threshold) {
                first_cluster.points = last_cluster.points.concat(first_cluster.points);
                this.detected_objects.pop();
            }
        }
    }

    private finalizeCluster(cluster: cluster) {
        if (cluster.points.length <= 1) return;

        let sum_radius = cluster.points.reduce((acc, p) => acc + p.radius, 0);
        let avg_radius = sum_radius / cluster.points.length;

        if (avg_radius < this.lidar_radius - 20) {
            this.detected_objects.push(cluster);
        }
    }

   private analyzeClusters() {
    this.detected_objects.forEach((item) => {
        const points = item.points.map(p => p.end_pos);
        const count = points.length;
        if (count === 0) return;

        
        let sumX = 0, sumY = 0;
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;

        points.forEach(p => {
            sumX += p.x;
            sumY += p.y;
            if (p.x < minX) minX = p.x;
            if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y;
            if (p.y > maxY) maxY = p.y;
        });

        const meanX = sumX / count;
        const meanY = sumY / count;

        
        item.min = { x: minX, y: minY };
        item.max = { x: maxX, y: maxY };

        
        let varXX = 0, varYY = 0, varXY = 0;
        points.forEach(p => {
            const dx = p.x - meanX;
            const dy = p.y - meanY;
            varXX += dx * dx;
            varYY += dy * dy;
            varXY += dx * dy;
        });

        varXX /= count;
        varYY /= count;
        varXY /= count;

        
        const trace = varXX + varYY;
        const det = (varXX * varYY) - (varXY * varXY);
        const term = Math.sqrt(Math.max(0, (trace * trace / 4) - det));
        
        const lambda1 = (trace / 2) + term; 
        const lambda2 = (trace / 2) - term; 
        
        const linearity = (lambda1 - lambda2) / (lambda1 + lambda2 + 1e-6); 
        const majorLength = 2 * Math.sqrt(lambda1) * Math.sqrt(3);          

        
        const WALL_LENGTH_THRESHOLD = 150; 
        const LINEARITY_THRESHOLD = 0.85;   

        if (majorLength > WALL_LENGTH_THRESHOLD && linearity > LINEARITY_THRESHOLD) {
            item.obstacle_type = ObstacleType.WALL;
        } else if (majorLength <= 40 && linearity < 0.6) {
            item.obstacle_type = ObstacleType.ROBOT; 
        } else{
            item.obstacle_type = ObstacleType.FIELD_OBSTACLE;
        }
    });
}

    private printLidarMap(){
       /*  this.lidar_array.forEach((item) => {
            //console.log("start x: " + item.start_pos.x + ", start y: " + item.start_pos.y + " end x: " + item.end_pos.x + ", end y: " + item.end_pos.y)
            //console.log("Radius: " + item.radius + ", Angle: " + item.angle)
        }) */

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

    

    render(ctx: CanvasRenderingContext2D, s_angle: number, e_angle: number, n_rays: number, l_radius: number){
        if(this.keys.p && !this.printed) {this.printLidarMap()};
        this.updateRobotPosition();
        this.updateLidarPosition();
        this.generateClusters();
        this.analyzeClusters();

        let is_moving = (this.keys.w || this.keys.s || this.keys.a || this.keys.d || this.keys.q || this.keys.e);
        if (is_moving) {
            this.analyzeLidarPoints();
            this.resampleParticles();
        }
        this.expected_position = this.getEstimatedPosition();


        if(s_angle != this.starting_angle || e_angle != this.ending_angle){
            this.starting_angle = s_angle;
            this.ending_angle = e_angle;
            console.log("trying to add new rays");
            this.addLidarRays(this.lidar_radius, this.num_rays);
        }

        if(n_rays != this.num_rays){
            this.num_rays = n_rays;
            console.log("trying to add new rays");
            this.addLidarRays(this.lidar_radius, this.num_rays);
        }

        if(l_radius != this.lidar_radius){
            this.lidar_radius = l_radius;
            console.log("trying to add new rays");
            this.addLidarRays(this.lidar_radius, this.num_rays);
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

        this.detected_objects.forEach((item) => {
        ctx.beginPath();
        ctx.rect(
            item.min.x, 
            item.min.y, 
            item.max.x - item.min.x, 
            item.max.y - item.min.y
        );

        // Assign stroke color based on obstacle type
        switch (item.obstacle_type) {
            case ObstacleType.WALL:
                ctx.strokeStyle = 'purple';
                break;
            case ObstacleType.ROBOT:
                ctx.strokeStyle = 'orange';
                break;
            case ObstacleType.FIELD_OBSTACLE:
                ctx.strokeStyle = 'cyan';
                break;
            default:
                ctx.strokeStyle = 'gray';
                break;
        }

        ctx.lineWidth = 3;
        ctx.stroke();
    });
        //ctx.rotate((45 * Math.PI) / 180);
        ctx.save();

        ctx.translate(this.position.x, this.position.y);

        let heading_rad = (this.position.heading * Math.PI) / 180;
        ctx.rotate(heading_rad);        
        ctx.fillStyle = 'black'; 
        ctx.fillRect(-this.size / 2, -this.size / 2, this.size, this.size);

        ctx.fillStyle = 'yellow';
        ctx.fillRect(0, -this.size / 4, this.size / 2, this.size / 2);

        
        ctx.restore();
        //ctx.rotate(-(45 * Math.PI) / 180);
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
            actual_y_element.innerText = `${this.position.y},`
        }

        const actual_heading_element = document.getElementById("actual-heading");
        if(actual_heading_element){
            actual_heading_element.innerText = `${this.position.heading.toFixed(1)}`
        }

        const expected_x_element = document.getElementById("expected-x");
        if(expected_x_element){
            expected_x_element.innerText = `${this.expected_position.x.toFixed(1)},`
        }

        const expected_y_element = document.getElementById("expected-y");
        if(expected_y_element){
            expected_y_element.innerText = `${this.expected_position.y.toFixed(1)}`
        }
        const expected_heading_element = document.getElementById("expected-heading");
        if(expected_heading_element){
            expected_heading_element.innerText = `${this.expected_position.heading.toFixed(1)}`
        }
    }
};