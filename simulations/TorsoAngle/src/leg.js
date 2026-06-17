import * as helpers from './helpers.js';

let thighLengthMm = 40;
let calfLengthMm = 120;
let servoDistanceMm = 46;
let springRateNmm = 0.5;

let frontCalfCompressionMm;
let rearCalfCompressionMm;

export class Leg
{
    constructor()
    {
        this.thighLength = thighLengthMm;
        this.calfLength = calfLengthMm;
        this.servoDistance = servoDistanceMm;
        this.thighAngle = null;       // kept for back-compat (front == rear)
        this.frontThighAngle = null;
        this.rearThighAngle = null;
        this.frontCalfCompression = null;
        this.rearCalfCompression = null;
        this.footPosition = {x: null, y: null};
        this.frontKneePosition = {x: null, y: null};
        this.rearKneePosition = {x: null, y: null};
        this.footReactionForce = {x: null, y: null};
        this.springRate = springRateNmm;
        this.legAngle = null;
    }

    _checkThigh(a)
    {
        if (a <= 90 || a > 180) throw new Error("Thigh angle out of range.");
    }

    // Single angle for both thighs (symmetric leg) -- back-compatible.
    setThighAngle(thighAngleDeg)
    {
        this.setThighAngles(thighAngleDeg, thighAngleDeg);
    }

    // Independent angle for each of the leg's two thighs (front-calf / rear-calf).
    setThighAngles(frontDeg, rearDeg)
    {
        this._checkThigh(frontDeg); this._checkThigh(rearDeg);
        this.frontThighAngle = frontDeg;
        this.rearThighAngle = rearDeg;
        this.thighAngle = frontDeg;   // representative value for legacy reads
    }

    setFrontCalfCompression(frontCalfCompressionMm)
    {
        this.frontCalfCompression = frontCalfCompressionMm;
    }

    setRearCalfCompression(rearCalfCompressionMm)
    {
        this.rearCalfCompression = rearCalfCompressionMm;
    }

    // Knee positions depend only on the thigh angle (not the calf compressions),
    // so they can be computed independently. Returns the local-frame positions and
    // also stores them on the instance.
    computeKneePositions()
    {
        if (this.frontThighAngle === null || this.rearThighAngle === null)
        {
            throw new Error("Thigh angles must be set before computing knee positions.");
        }

        const aF = helpers.degToRad(this.frontThighAngle - 90);
        const aR = helpers.degToRad(this.rearThighAngle - 90);

        this.frontKneePosition = {
            x: - this.servoDistance / 2 - this.thighLength * Math.sin(aF),
            y: this.thighLength * Math.cos(aF)
        };

        this.rearKneePosition = {
            x: this.servoDistance / 2 + this.thighLength * Math.sin(aR),
            y: this.thighLength * Math.cos(aR)
        };

        return { front: this.frontKneePosition, rear: this.rearKneePosition };
    }

    solveForwardKinematics()
    {
        if (this.thighAngle === null || this.frontCalfCompression === null || this.rearCalfCompression === null)
        {
            throw new Error("Thigh angle and calf compressions must be set before solving forward kinematics.");
        }

        let frontCalfLength = this.calfLength - this.frontCalfCompression;
        let rearCalfLength = this.calfLength - this.rearCalfCompression;

        this.computeKneePositions();

        let frontCalfCircle = new helpers.Circle(this.frontKneePosition.x, this.frontKneePosition.y, frontCalfLength);
        let rearCalfCircle = new helpers.Circle(this.rearKneePosition.x, this.rearKneePosition.y, rearCalfLength);

        let intersections = helpers.Circle.getCircleIntersection(frontCalfCircle, rearCalfCircle);
        if (intersections === null)
        {
            throw new Error("No intersection found for the calf circles.");
        }

        let footPosition = intersections[0].y > intersections[1].y ? intersections[0] : intersections[1];
        this.footPosition = footPosition;
    }

    calculateFootReactionForce()
    {
        if (this.footPosition.x === null || this.footPosition.y === null)
        {
            throw new Error("Foot position must be calculated before calculating foot reaction force.");
        }

        let frontCalfForceMag = this.frontCalfCompression * this.springRate;
        let rearCalfForceMag = this.rearCalfCompression * this.springRate;

        let frontCalfAngleRad = helpers.calculateAngleBetweenPoints(this.frontKneePosition, this.footPosition);
        let rearCalfAngleRad = helpers.calculateAngleBetweenPoints(this.rearKneePosition, this.footPosition);

        let frontCalfForce = {
            x: frontCalfForceMag * Math.cos(frontCalfAngleRad),
            y: frontCalfForceMag * Math.sin(frontCalfAngleRad)
        };

        let rearCalfForce = {
            x: rearCalfForceMag * Math.cos(rearCalfAngleRad),
            y: rearCalfForceMag * Math.sin(rearCalfAngleRad)
        };

        this.footReactionForce = {
            x: frontCalfForce.x + rearCalfForce.x,
            y: frontCalfForce.y + rearCalfForce.y
        };

        this.getReactionAngle();
        
    }

    getReactionAngle()
    {
        if (this.footReactionForce.x === null || this.footReactionForce.y === null)
        {
            throw new Error("Foot reaction force must be calculated before calculating reaction angle.");
        }
        
        let reactionAngle = helpers.radToDeg(Math.atan2(this.footReactionForce.y, this.footReactionForce.x));
        this.calculateLegAngle(reactionAngle);
        return reactionAngle;
    }

    calculateLegAngle(reactionAngle)
    {
        this.legAngle = reactionAngle - 90;
    }
}
