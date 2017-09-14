#ifndef CAMERA_H
#define CAMERA_H


struct CameraParameters {

    int    sx;
    int    sy;
    double fx;
    double fy;
    double cx;
    double cy;

};

struct PixelPoint {
    double u;
    double v;
};

struct MetricPoint {
    double x;
    double y;
};

struct AnglePoint {
    double a;
    double e;
};

class Camera {

public:
    Camera(const CameraParameters& params);

    void setParameters(const CameraParameters& params);

    /**
     * Take an input (u,v) in image coordinates, and produce
     * an output in the metrix coordinates (X,Y).
     */
    MetricPoint pixel2metric(PixelPoint p);
    MetricPoint pixel2metric(double u, double v);

    /* The inverse of the above */
    PixelPoint metric2pixel(MetricPoint p);
    PixelPoint metric2pixel(double x, double y);


    /**
         * Take an input (X,Y) in metric coordinates, and produce
         * an output in azimuth-elevation angles.
         * This represents a rotation of -azimuth around the camera
         * frame Y axis, followed by a rotation of -elevation around
         * the current X axis.
         * Thus the azimuthal angle grows to the left of the visual field and
         * the elevation angle grows to the above of the visual field.
         * Angles are represented in degrees
         */
    AnglePoint metric2angle(MetricPoint p);
    AnglePoint metric2angle(double x, double y);

    /** The inverse of the above */
    MetricPoint angle2metric(AnglePoint p);
    MetricPoint angle2metric(double a, double e);

    /**
         * Take an input (u,v) in pixel coordinates, and produce
         * an output in azimuth-elevation angles.
         * This represents a rotation of azimuth around the camera
         * frame Y axis, followed by a rotation of elevation around
         * the current X axis.
         * Angles are represented in degrees
         */
    AnglePoint pixel2angle(PixelPoint p);
    AnglePoint pixel2angle(double u, double v);

    /** The inverse of the above */
    PixelPoint angle2pixel(AnglePoint p);
    PixelPoint angle2pixel(double a, double e);


    /**
         * Take an input (u,v) in pixel coordinates, and produce
         * an output in normalized pixel coordinates.
         * The coordinates become independent of image size
         */
    PixelPoint pixel2norm(PixelPoint p);
    PixelPoint pixel2norm(double u, double v);

    /* The inverse of the above */
    PixelPoint norm2pixel(PixelPoint p);
    PixelPoint norm2pixel(double u, double v);


    /**
         * Take an input (x,y) normalized pixel coordinates, and produce
         * an output in azimuth-elevation angles.
         * Angles are represented in degrees
         */
    AnglePoint norm2angle(PixelPoint p);
    AnglePoint norm2angle(double u, double v);

    /**
         * Take an input (x,y) normalized pixel coordinates, and produce
         * an output in metrix X,Y coordinates.
         */
    MetricPoint norm2metric(PixelPoint p);
    MetricPoint norm2metric(double u, double v);


    inline int get_height() {return sy;}
    inline int get_width() {return sx;}
    inline double get_fx() {return fx;}
    inline double get_fy() {return fy;}
    inline double get_cx() {return cx;}
    inline double get_cy() {return cy;}

protected:
    int    sx;     /** the camera width in pixels (image width at calibration)*/
    int    sy;     /** the camera height in pixels (image height at calibration)*/
    double fx;     /** the horizontal focal distance in horizontal pixel units  */
    double fy;     /** the vertical focal distance in vertical pixel units */
    double cx;     /** the horizontal center of coordinates in horizontal pixel units */
    double cy;     /** the vertical center of coordinates in vertical pixel units */

};

#endif // CAMERA_H
