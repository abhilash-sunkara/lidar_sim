import type { point_vector } from "./raycast_utils";

const MovementType = {
    UP: "UP",
    DOWN : "DOWN",
    LEFT : "LEFT",
    RIGHT : "RIGHT"
}

export class OpponentRobot{
    private position: point_vector;
    private size: number;
    private mt = MovementType.UP;

    constructor(){
        this.position = {x: 120, y: 0};
        this.size = 90;
    }

    private updateRobotPosition() {
        if(this.mt == MovementType.UP){
            this.position.y += 1
        }
        if(this.mt == MovementType.LEFT){
            this.position.x -= 1
        }
        if(this.mt == MovementType.DOWN){
            this.position.y -= 1
        }
        if(this.mt == MovementType.RIGHT){
            this.position.x += 1
        }

        if(this.mt == MovementType.UP && this.position.y > 120){
            this.mt = MovementType.LEFT;
        }

        if(this.mt == MovementType.LEFT && this.position.x < -120){
            this.mt = MovementType.DOWN;
        }

        if(this.mt == MovementType.DOWN && this.position.y < -120){
            this.mt = MovementType.RIGHT;
        }

        if(this.mt == MovementType.RIGHT && this.position.x > 120){
            this.mt = MovementType.UP;
        }
    }

    get_pos_object(){
        return this.position;
    }

    render(ctx: CanvasRenderingContext2D){
        this.updateRobotPosition();
        ctx.fillStyle = 'green'; 
        ctx.fillRect(this.position.x - this.size / 2, this.position.y - this.size / 2, this.size, this.size);
    }
}
