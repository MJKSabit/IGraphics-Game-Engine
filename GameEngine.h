#include <bits/stdc++.h>

const double PI = acos(-1.0);
const double WIDTH = 800, HEIGHT = 500;

const double FORCE_FACTOR = 10;

const int FPS = 60;
const double SecondsPerFrame = 1/double(FPS);

double camX = 0;


/*********************************************

Everything from POINT

**********************************************/

class Point
{
    double x, y;
public:
    Point(double x_=0, double y_=0)
    {
        this->x = x_;
        this->y = y_;
    }
    void setXY(const double x_=0, const double y_=0)
    {
        this->x = x_;
        this->y = y_;
    }
    void setRTheta(const double r=0, const double theta=0, const Point pole = Point(0, 0))
    {
        this->x = r*cos(theta) + pole.getX();
        this->y = r*sin(theta) + pole.getY();
    }

    void Transform(const Point origin)
    {
        this->x = this->x - origin.getX();
        this->y = this->y - origin.getY();
    }
    void Rotate(const double theta)
    {
        const double old_x = this->x, old_y = this->y;

        this->x = old_x*cos(theta) + old_y*sin(theta);
        this->y = old_y*cos(theta) - old_x*sin(theta);
    }
    void Rotate(const double theta, const Point origin)
    {
        const double dx = this->x-origin.getX(), dy = this->y-origin.getY();

        this->setRTheta(sqrt(dx*dx+dy*dy), atan2(dy, dx)+theta, origin);
    }

    double getX() const
    {
        return this->x;
    }
    double getY() const
    {
        return this->y;
    }

    double distance (const Point comp) const;
};

double Point::distance(const Point compare) const
{
    double delX = this->x-compare.getX();
    double delY = this->y-compare.getY();

    return sqrt(delX*delX+delY*delY);
}



/*********************************************************************

Everything for LineSegment

**********************************************************************/


class LineSegment
{
    Point initialPoint, endPoint;
    double theta;
public:
    LineSegment(const Point one, const Point two) {
        this->theta = atan2(two.getY()-one.getY(), two.getX()-one.getX());
        this->initialPoint = one;
        this->endPoint = two;
    }

    double getSlopeAngle () const
    {
        return this->theta;
    }
    Point getInitialPoint () const
    {
        return this->initialPoint;
    }
    Point getEndPoint () const
    {
        return this->endPoint;
    }

    double getLength() const
    {
        double dy = endPoint.getY()-initialPoint.getY();
        double dx = endPoint.getX()-initialPoint.getX();
        return sqrt(dx*dx+dy*dy);
    }

    void draw(double x_origin = 0) const
    {
        double new_x1 = this->initialPoint.getX()-x_origin;
        double new_x2 = this->endPoint.getX()-x_origin;

        if(-100<=new_x1 && new_x1<=WIDTH || 0<=new_x1 && new_x1<=WIDTH) iLine(new_x1, this->initialPoint.getY(), new_x2, this->endPoint.getY());
    }
};



/*********************************************

Everything from VECTOR

**********************************************/

class Vector{
    double x, y;

public:
    Vector()
    {
        x = y = 0;
    }

    Vector(double a, double b)
    {
        this->x = a;
        this->y = b;
    }

    Vector(Point initial, Point end)
    {
        this->x = end.getX()-initial.getX();
        this->y = end.getY()-initial.getY();
    }

    Vector(LineSegment line)
    {
        this->x = line.getEndPoint().getX()-line.getInitialPoint().getX();
        this->y = line.getEndPoint().getY()-line.getInitialPoint().getY();
    }

    Vector(double theta) /// Unit Vector
    {
        this->x = cos(theta);
        this->y = sin(theta);
    }

    Vector add(Vector old)
    {
        return Vector(x + old.getX(), y + old.getY());
    }

    Vector inverse()
    {
        return Vector(-x, -y);
    }

    Vector substruct(Vector old)
    {
        return add(old.inverse());
    }

    Vector multiply(double M)
    {
        return Vector(M*x, M*y);
    }

    double multiplyDot(const Vector rs)
    {
        return x*rs.getX() + y*rs.getY();
    }

    double multiplyCross(const Vector rs) /// AntiClockWise == +
    {
        return x*rs.getY()-y*rs.getX();
    }

    double getValue() const
    {
        return sqrt(x*x+y*y);
    }

    double getX() const { return this->x; }
    double getY() const { return this->y; }

    double getUnitVectorAngle() const
    {
        return atan2(y, x);
    }

};


class Motion{
    Vector g = Vector(0, -9.80665);
    const double Mu_k = 0.1;

    double mass, movementAngle, surfaceAngle;
    int onSurface = 0;

    Vector netForce, accelaration,velocity, displacement;
    Vector frictionForce, reactionForce, surfaceForce;
    double engineForce = 0;

    Vector Weight;

public:
    Vector cf;

    Motion(Vector prevPosition, double m)
    {
        mass = m;
        accelaration = Vector(0.0, 0.0);
        velocity = Vector(0.0, 0.0);
        displacement = prevPosition;
        Weight = g.multiply(mass*FORCE_FACTOR);
    }

    double getEngineForce()
    {
        return engineForce;
    }

    double getFrictionForce()
    {
        return frictionForce.getValue();
    }

    double getSurfaceAngle()
    {
        return surfaceAngle;
    }

    void setOnSurface(int a, double theta=0)
    {
        onSurface = a;
        surfaceAngle = theta;
    }

    void setVelocity(Vector v)
    {
        velocity = v;
    }

    void setMass(double m)
    {
        mass = m;
    }

    double getMass() const
    {
        return mass;
    }

    void setForces(std::vector <Vector> forces = {})
    {
        netForce = Vector(0, 0);

        for(int i=0; i<forces.size(); i++) netForce = netForce.add(forces[i].multiply(FORCE_FACTOR));
    }

    void addForces (Vector vct)
    {
        netForce = netForce.add(vct.multiply(FORCE_FACTOR));
    }

    void setEngineForces(double Magnitude)
    {
        engineForce = Magnitude*FORCE_FACTOR;
    }

    void addEngineForce(double magnitude)
    {
        engineForce += magnitude*FORCE_FACTOR;
    }

    double getXVelocity()
    {
        return velocity.getX();
    }

    void activate()
    {
        Vector currentForce = Weight.add(netForce);

        frictionForce = Vector();

        // printf("Net Force %.2f %.2f\n", netForce.getX(), netForce.getY());


        if(onSurface)
        {
            Vector unitVectorToSurface =  Vector(surfaceAngle); /// Unit Vector through Surface
            Vector unitVectorPerpendicular = Vector(surfaceAngle+PI/2); /// Reaction Force is Upward

            surfaceForce = unitVectorToSurface.multiply(unitVectorToSurface.multiplyDot(currentForce));
            reactionForce = unitVectorPerpendicular.multiply(unitVectorPerpendicular.multiplyDot(currentForce)); /// For friction

            if(unitVectorPerpendicular.multiplyDot(currentForce)<0) /// Reaction Force Downward
            {
                int velocity_sign = velocity.multiplyDot(unitVectorToSurface)>0 ? 1 : velocity.multiplyDot(unitVectorToSurface)==0 ? 0 : -1;
                int friction_sign = -velocity_sign;

                velocity = unitVectorToSurface.multiply(velocity.multiplyDot(unitVectorToSurface));

                frictionForce = unitVectorToSurface.multiply(reactionForce.getValue()*friction_sign*Mu_k);

                surfaceForce = surfaceForce.add(frictionForce);
                surfaceForce = surfaceForce.add(unitVectorToSurface.multiply(engineForce));

                currentForce = surfaceForce;
                reactionForce = reactionForce.multiply(-1);
            }
            else
            {
                if(unitVectorPerpendicular.multiplyDot(velocity)<0) /// Velocity is Downward
                {
                    velocity = unitVectorToSurface.multiply(velocity.multiplyDot(unitVectorToSurface));
                }
                currentForce = surfaceForce.add(reactionForce);
            }
        }

        accelaration = currentForce.multiply(1/mass);
        velocity = velocity.add(accelaration.multiply(SecondsPerFrame));



        displacement = displacement.add(velocity.multiply(SecondsPerFrame));
        cf = currentForce;
        //printf("F: %.1f %.1f V: %.1f %.1f\n", netForce.getX(), netForce.getY(), velocity.getX(), velocity.getY());

    }

    void collusionLine(LineSegment line, double radius)
    {
        Vector vct_line = Vector(line);
        Vector vct_ray = Vector(line.getInitialPoint(), getCenter());

        double d = abs(vct_ray.multiplyCross(vct_line))/vct_line.getValue();

        if(d-radius<=1.0) /// Collide
        {
            this->setOnSurface(1, line.getSlopeAngle());
            if(d<radius)
            {
                double x, y, diff, ang;

                diff = radius-d;
                ang = line.getSlopeAngle() + PI/2;
                x = this->getMovementX() + diff*cos(ang);
                y = this->getMovementY() + diff*sin(ang);

                this->setCenter(Point(x, y));
            }
        }
        else
        {
            this->setOnSurface(0);
        }
    }

    void collusion2Line(LineSegment first, LineSegment second, double radius)
    {
        double alpha = first.getSlopeAngle();
        double beta = second.getSlopeAngle();

        Point intersection = first.getEndPoint();

        LineSegment ray(intersection, this->getCenter());

        double theta = ray.getSlopeAngle();
        if(theta<0) theta += 2*PI;

        //printf("A %.1f B %.1f T %.1f\n", alpha*180/PI, beta*180/PI, theta*180/PI);

        double cmp = (alpha+beta+PI)/2, d, angle;
        LineSegment line = LineSegment(Point(-100, -100), Point(-10, -10));

        if (beta>=alpha) /// \_/
        {
            if(abs(cmp-alpha)<=1e-6)
            {
                if(this->getXVelocity()>0) line = second;
                else line = first;
            }
            else if(theta>cmp)
            {
                line = first;
            }
            else{
                line = second;
            }
        }
        else
        {
            if(theta>alpha+PI/2) line = first;
            else if(theta<beta+PI/2) line = second;
            else{
                if(cmp<=theta) angle = this->getXVelocity() > 0 ? first.getSlopeAngle() : theta-PI/2;
                else angle =  this->getXVelocity() < 0 ? second.getSlopeAngle() : theta-PI/2;

                d = ray.getLength();
                if(d-radius<=1.0)
                {
                    this->setOnSurface(1, angle);
                    if(d<radius) moveCenter(radius-d, angle+PI/2);
                }
                else
                {
                    this->setOnSurface(0);
                }
                return;
            }
        }

        this->collusionLine(line, radius);
    }

    double moveCenter(double diff, double ang)
    {
        double x, y;

        x = this->getMovementX() + diff*cos(ang);
        y = this->getMovementY() + diff*sin(ang);

        this->setCenter(Point(x, y));
    }

    double getMovementX() const
    {
        return displacement.getX();
    }

    double getMovementY() const
    {
        return displacement.getY();
    }

    Point getCenter() const
    {
        return Point(displacement.getX(), displacement.getY());
    }

    void setCenter(Point new_cen)
    {
        displacement = Vector(Point(0,0), new_cen);
    }
};
