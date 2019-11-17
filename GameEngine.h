#include <bits/stdc++.h>

double PI = acos(-1.0);
double WIDTH = 800, HEIGHT = 500;

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
    Vector g = Vector(0, -9.80665*10);
    const double Mu_k = 0.1;

    double mass, movementAngle, surfaceAngle;
    int onSurface = 0;

    Vector netForce, accelaration,velocity, displacement;

    Vector Weight;

public:
    Motion(Vector prevPosition, double m)
    {
        mass = m;
        accelaration = Vector(0.0, 0.0);
        velocity = Vector(0.0, 0.0);
        displacement = prevPosition;
        Weight = g.multiply(mass);
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

    void setEngineForces(std::vector <Vector> forces)
    {
        netForce = Vector(0, 0);

        for(int i=0; i<forces.size(); i++) netForce = netForce.add(forces[i]);
    }

    void addEngineForce(double magnitude)
    {
        netForce = netForce.add(Vector(magnitude, 0));
    }

    double getXVelocity()
    {
        return velocity.getX();
    }

    void activate()
    {
        Vector currentForce = Weight;
        Vector frictionForce, reactionForce;

        if(onSurface)
        {
            Vector unit =  Vector(surfaceAngle);

            //currentForce = currentForce.add(netForce);

            currentForce = unit.multiply(unit.multiplyDot(currentForce));
            reactionForce = netForce.substruct(currentForce); /// For friction

            int velocity_sign = velocity.multiplyDot(unit)>0 ? 1 : velocity.multiplyDot(unit)==0 ? 0 : -1;
            int friction_sign = -velocity_sign;

            frictionForce = unit.multiply(reactionForce.getValue()*friction_sign*Mu_k);

            currentForce = currentForce.add(unit.multiply(netForce.getX()));

            velocity = unit.multiply(velocity.multiplyDot(unit));
        }

        printf("FRICTION %f %f\n", frictionForce.getX(), frictionForce.getY());
        currentForce = currentForce.add(frictionForce); /// Not (-) as already added Direction

        accelaration = currentForce.multiply(1/mass);
        velocity = velocity.add(accelaration.multiply(SecondsPerFrame));



        displacement = displacement.add(velocity.multiply(SecondsPerFrame));

        //printf("F: %.1f %.1f V: %.1f %.1f\n", netForce.getX(), netForce.getY(), velocity.getX(), velocity.getY());
        camX = getMovementX()-WIDTH/2;

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
