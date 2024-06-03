package org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous;

public class Paths {

    public static double[] startPoint = {0,0};
    public static double[] targetPoint = {40,50};
    public static double[] intermediatePoint = {10, 30};

    static double movementTestY = 38;
    static double movementTestX = -60;

    public enum Path {
        FirstPath(new double[][]{startPoint, targetPoint}),
        CurvyFirstPath(new double[][]{startPoint, intermediatePoint, targetPoint}),
        Path1(new double[][]{{0,0}, {41,-8}}),
        Path2(new double[][]{{41, -8}, {69,-8}, {72,-66}, {110, -67}}),
        Path3(new double[][]{{110, -67}, {138, -66}}),
        Path4(new double[][]{{114,324}, {173,332}}),
        Path5(new double[][]{{173,332}, {170,451}}),
        Path6(new double[][]{{170,451}, {50, 442}}),
        Path7(new double[][]{{50, 442}, {52,378}}),
        Path8(new double[][]{{52,378}, {-6,370}}),
        Path9(new double[][]{{-6,370}, {-4, 270}}),
        Score,
        WaitForCone,
        Transfer,
        //TESTING
        Forward80(new double[][]{{0,-130}, {0,0}}), Backwards80(new double[][]{{0,0}, {0, -130}}),
        MTPath1(new double[][]{{0, 0}, {0, Paths.movementTestY}, {movementTestX, Paths.movementTestY}, {movementTestX, 0}}), MTPath2(new double[][]{{movementTestX, 0}, {movementTestX, -Paths.movementTestY}, {2*movementTestX, -Paths.movementTestY}, {2*movementTestX, 0}}),
        MTPath3(new double[][]{{2*movementTestX, 0}, {2*movementTestX, Paths.movementTestY}, {3*movementTestX, Paths.movementTestY}, {3*movementTestX, 0}}), MTPath4(new double[][]{{3*movementTestX, 0}, {3*movementTestX, -Paths.movementTestY}, {2*movementTestX, -Paths.movementTestY}, {2*movementTestX, 0}}),
        MTPath5(new double[][]{{2*movementTestX, 0}, {2*movementTestX, Paths.movementTestY}, {movementTestX, Paths.movementTestY}, {movementTestX, 0}}), MTPath6(new double[][]{{movementTestX, 0}, {movementTestX, -Paths.movementTestY}, {0, -Paths.movementTestY}, {0, 0}});

        private double[][] pathPoints;
        private Paths.Function[] path;
        private PathType pathType;

        public double[] endPoint;

        public double x, y, t = 0;
        public double dx, dy, d2x, d2y;

        public boolean curvy;
        public boolean targeting = false;

        public double targetingPower = .2;
        public double tIncrementCM = .2;
        public double tIncrement;

        public enum PathType{
            Curve, Line, Orbit, NONE
        }

        Path(double[][] pathPoints){
            if(pathPoints.length > 2){
                pathType = PathType.Curve;
            }else{
                pathType = PathType.Line;
            }
            construct(pathPoints, pathType);
        }

        public void construct(double[][] pathPoints, PathType pathType){
            this.pathPoints = pathPoints;
            this.pathType = pathType;
        }

        public void compile(){
            calculatePath();
            getTIncrement();
            resetPath();
        }

        public void resetPath(){
            x = getX(0);
            y = getY(0);
            t = 0;
            targeting = false;
            updatePathDerivatives();
        }

        public void compile( double tIncrement){
            calculatePath();
            this.tIncrement = tIncrement;
        }

        Path(){
            this.pathPoints = new double[][]{{0},{0}};
            this.pathType = PathType.NONE;
        }

        private void calculatePath(){
            switch (pathType){
                default:
                case Line:
                    path = Paths.getLine(pathPoints);
                    curvy = false;
                    break;

                case Curve:
                    path = Paths.getBezierCurve(pathPoints);
                    curvy = true;
                    break;

                case Orbit:
                    path = Paths.getOrbit(pathPoints);
                    curvy = true;
                    break;
            }
            x = getX(0);
            y = getY(0);
            setEndPoint(getX(1), getY(1));
        }


        private void getTIncrement(){
//            integrate to find the length of the path
            double dt = .001;
            double td = 1/dt;
            double pathLength = 0;
            for(int t = 0; t <= td; t++){
                double f = t * dt;
                pathLength += Math.hypot(getDX(f), getDY(f)) * dt;
            }
            tIncrement = tIncrementCM/pathLength;
//            tIncrement = .003;
            //tIncrement .003 for 200cm

        }

        public double distanceToTarget(){
            return Math.hypot(x-endPoint[0], y-endPoint[1]);
        }

        public void setEndPoint(double x, double y){
            endPoint = new double[]{x, y};
        }

        public void setTargetingPower(double p){
            targetingPower = p;
        }

        //getting specific points on path
        public double[] getPoint(double t){
            return new double[]{getX(t), getY(t)};
        }

        public double getX(double t) {
            if(t > 1){
                return getX(1);
            }
            return path[0].get(t);
        }

        public double getY(double t){
            if(t > 1){
                return getY(1);
            }
            return path[1].get(t);
        }

        public double getDX(double t){
            if(t > 1){
                return getDX(1);
            }
            double dx = path[2].get(t);
            return (dx == 0) ? .00000001 : dx;
        }

        public double getDY(double t){
            if(t > 1){
                return getDY(1);
            }
            return path[3].get(t);
        }

        public double getD2X(double t){
            if(t > 1){
                return getD2X(1);
            }
            double d2x = path[4].get(t);
            return (d2x == 0) ? .000000001 : d2x;
        }

        public double getD2Y(double t) {
            if(t > 1){
                return getD2Y(1);
            }
            return path[5].get(t);
        }

        //updating current class
        public void updatePathDerivatives(){
            updateDX(t);
            updateDY(t);
            updateD2X(t);
            updateD2Y(t);
        }

        public double updateX(double t){
            x = getX(t);
            return x;
        }

        public double updateY(double t){
            y = getY(t);
            return y;
        }

        public double updateDX(double t){
            dx = getDX(t);
            return dx;
        }

        public double updateDY(double t){
            dy = getDY(t);
            return dy;
        }

        public double updateD2X(double t){
            d2x = getD2X(t);
            return d2x;
        }

        public double updateD2Y(double t){
            d2y = getD2Y(t);
            return d2y;
        }
    };

    //Input points of a bezier curve in the order you want them to be calculated
    public static Paths.Function[] getBezierCurve(double[][] points){
        Paths.Function[] spline = new Paths.Function[6];
        int n = points.length-1;
        //x of point
        spline[0] = (u) -> {
            double x = 0;
            for(int i = 0; i <= n; i++){
                x += B(n, i, u) * points[i][0];
            }
            return x;
        };
        //y of point
        spline[1] = (u) -> {
            double y = 0;
            for(int i = 0; i <= n; i++){
                y += B(n, i, u) * points[i][1];
            }
            return y;
        };
        //x' of point
        spline[2] = (u) -> {
            double dx = 0;
            for(int i = 0; i <= n-1; i++){
                dx += B(n-1, i, u) * ((n) * (points[i+1][0] - points[i][0]));
            }
            return dx;

        };
        //y' of point
        spline[3] = (u) -> {
            double dy = 0;
            for (int i = 0; i <= n - 1; i++) {
                dy += B(n - 1, i, u) * ((n) * (points[i + 1][1] - points[i][1]));
            }
            return dy;
        };

        //x'' of point
        spline[4] = (u) -> {
            double d2x = 0;
            for(int i = 0; i <= n-2; i++){
                d2x += B(n-2, i, u) * ((n) *(n-1) * (points[i+2][0] -2 * points[i+1][0] + points[i][0]));
            }
            return d2x;
        };
        //y'' of point
        spline[5] = (u) -> {
            double d2y = 0;
            for(int i = 0; i <= n-2; i++){
                d2y += B(n-2, i, u) * ((n) *(n-1) * (points[i+2][1] -2 * points[i+1][1] + points[i][1]));
            }
            return d2y;
        };

        return spline;
    }

    //takes 2 points, start and end
    public static Paths.Function[] getLine(double[][] points){
        //t goes between 0 and 1, 1 being the endpoint of the graph
        double xDiff = points[1][0] - points[0][0];
        double yDiff = points[1][1] - points[0][1];
        Paths.Function[] line = new Paths.Function[6];
        line[0] = (t) -> t*xDiff+points[0][0];
        line[1] = (t) -> t*yDiff+points[0][1];
        line[2] = (t) -> xDiff;
        line[3] = (t) -> yDiff;
        line[4] = (t) -> 0;
        line[5] = (t) -> 0;
        return line;
    }

    //takes start point, center of rotation, and fraction of circle wanted to be rotated (make sure to include direction with +/-
    public static Paths.Function[] getOrbit(double[][] points){

        double[] center = points[0];
        double[] start = points[1];
        double fraction = points[2][0];
        double radius = Math.sqrt(Math.pow(center[0] - start[0], 2) + Math.pow(center[1] - start[1], 2));

        Paths.Function[] arc = new Paths.Function[2];

        arc[0] = (t) -> center[0] + Math.cos(t*2*Math.PI*fraction)*radius;
        arc[1] = (t) -> center[1] + Math.sin(t*2*Math.PI*fraction)*radius;

        return arc;
    }

    //faster
    private static double fact(int n){
        switch (n){
            case 0:
            case 1: return 1;
            case 2: return 2;
            case 3: return 6;
            case 4: return 24;
            case 5: return 120;
            case 6: return 720;
            default: return 0;
        }
    }

    private static double B(int n, int i, double u){ return fact(n)/(fact(i) * fact(n-i)) * (double)Math.pow(u, i) * (double)Math.pow(1-u, n-i); }

    public interface Function{ double get(double t);}
}
