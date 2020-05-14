package senzorski3;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.ThreadLocalRandom;
import java.util.stream.Collectors;

class Point {
	private int id;
	private double x;
	private double y;
	private double xpr;
	private double ypr;
	private double error;
	private boolean anchor;
	private int cost;
	private Set<Anchor> anchorsInRange;
	
	public Point(int id, double x, double y) {
		this.id = id;
		this.x=x;
		this.y=y;
		this.xpr = 0;
		this.ypr = 0;
		this.error = 0;
		this.anchor = false;
		this.cost = 0;
		this.anchorsInRange = new TreeSet<>(new Comparator<Anchor>() {
			@Override
			public int compare(Anchor awd1, Anchor awd2) {
				if(awd1.getDistance()>awd2.getDistance()) return 1;
				if(awd1.getDistance()<awd2.getDistance()) return -1;
				else return 0;
			}
		});
	}



	public double getX() {
		return x;
	}



	public double getY() {
		return y;
	}




	public double getXpr() {
		return xpr;
	}



	public void setXpr(double xpr) {
		this.xpr = xpr;
	}



	public double getYpr() {
		return ypr;
	}



	public void setYpr(double ypr) {
		this.ypr = ypr;
	}



	public double getError() {
		return error;
	}



	public void setError(double error) {
		this.error = error;
	}


	public void setAnchor(boolean anchor) {
		this.anchor = anchor;
	}

	public Set<Anchor> getAnchorsInRange() {
		return anchorsInRange;
	}



	public boolean isAnchor() {
		return anchor;
	}



	public int getCost() {
		return cost;
	}



	public void setCost(int cost) {
		this.cost = cost;
	}



}

class Anchor{
	private Point p;
	private double distance;
	
	
	public Anchor(Point p, double distance) {
		this.p=p;
		this.distance=distance;
	}

	public Point getP() {
		return p;
	}


	public double getDistance() {
		return distance;
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}
	
}

public class Field{
	private int size;
	private int numPoints;
	private int anchorPercent;
	private int radiusRange;
	private int errorChancePercentage;
	private List<Point> points;
	
	public Field(int size, int numPoints, int anchorPercent, int radiusRange, int errorChancePercentage) {
		this.size=size;
		this.numPoints = numPoints;
		this.anchorPercent = anchorPercent;
		this.radiusRange = radiusRange;
		this.errorChancePercentage = errorChancePercentage;
		this.points = new ArrayList<>();
	}
	
	public double getNonAnchorsSize() {
		return (numPoints-(numPoints*(anchorPercent/100.0)));
	}
	
	public double getAnchorsSize() {
		return (numPoints*(anchorPercent/100.0));
	}
	
	
	public void generateFieldAndPoints() {
		
		for(int i = 0; i < numPoints;i++) {
			double x = ThreadLocalRandom.current().nextDouble(0, size+1);
			double y = ThreadLocalRandom.current().nextDouble(0, size+1);
			Point p = new Point(i+1,x,y);
			if(i>=getNonAnchorsSize()) {
				p.setAnchor(true);
			}
			points.add(p);
			
		}
		Random r = new Random();
		for(int i = 0;i<getNonAnchorsSize();i++) {
			for(int j = 0;j<getAnchorsSize();j++) {
				Point nonAnchor = points.get(i);
				Point anchor = points.get(points.size()-1-j);
				double distance = Math.sqrt(Math.pow((nonAnchor.getX()-anchor.getX()), 2) + Math.pow((nonAnchor.getY()-anchor.getY()), 2));
				//double distanceWithError = ThreadLocalRandom.current().nextDouble(distance-(distance*(10/100.0)), distance+(distance*(10/100.0)));
				double distanceWithError = (distance-(distance*(errorChancePercentage/100.0))) + ((distance+(distance*(errorChancePercentage/100.0))) - (distance-(distance*(errorChancePercentage/100.0)))) * r.nextDouble();
				if(distanceWithError<=radiusRange) {
					nonAnchor.getAnchorsInRange().add(new Anchor(anchor, distanceWithError));
				}
			}
		}
	}
	
	public void nonIterative() {	
		float ALE = 0;
		int count = 0;
		for(int i = 0;i<points.size();i++) {
			if(points.get(i).getAnchorsInRange().size()>=3) {
				List<Anchor> awd = points.get(i).getAnchorsInRange().stream().limit(3).collect(Collectors.toList());
				Anchor awd1 = awd.get(0);
				Anchor awd2 = awd.get(1);
				Anchor awd3 = awd.get(2);
				List<Double> newCoordinates = Trilateration(awd1,awd2,awd3);
				points.get(i).setXpr(newCoordinates.get(0));
				points.get(i).setYpr(newCoordinates.get(1));
				double error = Math.sqrt(Math.pow((points.get(i).getX()-newCoordinates.get(0)), 2) + Math.pow((points.get(i).getY()-newCoordinates.get(1)), 2));
				points.get(i).setError(error);
				ALE+=error;
				count++;
			}
		}
		System.out.println("Non-iterative: ALE: " + ALE/count + ", Localized nodes(%): " + (count*100)/getNonAnchorsSize() + "%");
		
	}
	
	public void iterativeMostRelevant() {
		List<Point> points_cloned = new ArrayList<>(points);
		Random r = new Random();
		int count = 0;
		float ALE = 0;
		for(int i = 0 ; i<points_cloned.size();i++) {
			if(points_cloned.get(i).getAnchorsInRange().size()>=3 && !points_cloned.get(i).isAnchor()) {
				List<Anchor> awd = points_cloned.get(i).getAnchorsInRange().stream().sorted(Comparator.comparing((Anchor a) -> a.getP().getCost()).thenComparing(Anchor::getDistance)).limit(3).collect(Collectors.toList());
				Anchor awd1 = awd.get(0);
				Anchor awd2 = awd.get(1);
				Anchor awd3 = awd.get(2);
				List<Double> newCoordinates = Trilateration(awd1,awd2,awd3);
				points_cloned.get(i).setXpr(newCoordinates.get(0));
				points_cloned.get(i).setYpr(newCoordinates.get(1));
				double error = Math.sqrt(Math.pow((points_cloned.get(i).getX()-newCoordinates.get(0)), 2) + Math.pow((points_cloned.get(i).getY()-newCoordinates.get(1)), 2));
				points_cloned.get(i).setError(error);
				points_cloned.get(i).setCost(awd1.getP().getCost() + awd2.getP().getCost() + awd3.getP().getCost() + 1);
				points_cloned.get(i).setAnchor(true);
				for(int j=0;j<points_cloned.size();j++) {
					if(!points_cloned.get(j).isAnchor() && i!=j) {
 						double distance = Math.sqrt(Math.pow((points_cloned.get(j).getX()-points_cloned.get(i).getX()), 2) + Math.pow((points_cloned.get(j).getY()-points_cloned.get(i).getY()), 2));
						//double distanceWithError = ThreadLocalRandom.current().nextDouble(distance-(distance*(10/100.0)), distance+(distance*(10/100.0)));
						double distanceWithError = (distance-(distance*(errorChancePercentage/100.0))) + ((distance+(distance*(errorChancePercentage/100.0))) - (distance-(distance*(errorChancePercentage/100.0)))) * r.nextDouble();
						if(distanceWithError<=radiusRange) {
							points_cloned.get(j).getAnchorsInRange().add(new Anchor(points_cloned.get(i), distanceWithError));
						}
					}
				}
				i=0;
				count++;
				ALE+=error;
			}
		}
		System.out.println("Iterative three most relevant: ALE: " + ALE/count + ", Localized nodes(%): " + (count*100)/getNonAnchorsSize() + "%");
		
	}
	
	public void iterativeThreeClosest() {
		List<Point> points_cloned = new ArrayList<>(points);
		Random r = new Random();
		int count = 0;
		float ALE = 0;
		for(int i = 0 ; i<points_cloned.size();i++) {
			if(points_cloned.get(i).getAnchorsInRange().size()>=3 && !points_cloned.get(i).isAnchor()) {
				List<Anchor> awd = points_cloned.get(i).getAnchorsInRange().stream().limit(3).collect(Collectors.toList());
				Anchor awd1 = awd.get(0);
				Anchor awd2 = awd.get(1);
				Anchor awd3 = awd.get(2);
				List<Double> newCoordinates = Trilateration(awd1,awd2,awd3);
				points_cloned.get(i).setXpr(newCoordinates.get(0));
				points_cloned.get(i).setYpr(newCoordinates.get(1));
				double error = Math.sqrt(Math.pow((points_cloned.get(i).getX()-newCoordinates.get(0)), 2) + Math.pow((points_cloned.get(i).getY()-newCoordinates.get(1)), 2));
				points_cloned.get(i).setError(error);
				points_cloned.get(i).setAnchor(true);
				for(int j=0;j<points_cloned.size();j++) {
					if(!points_cloned.get(j).isAnchor() && i!=j) {
 						double distance = Math.sqrt(Math.pow((points_cloned.get(j).getX()-points_cloned.get(i).getX()), 2) + Math.pow((points_cloned.get(j).getY()-points_cloned.get(i).getY()), 2));
						//double distanceWithError = ThreadLocalRandom.current().nextDouble(distance-(distance*(10/100.0)), distance+(distance*(10/100.0)));
						double distanceWithError = (distance-(distance*(errorChancePercentage/100.0))) + ((distance+(distance*(errorChancePercentage/100.0))) - (distance-(distance*(errorChancePercentage/100.0)))) * r.nextDouble();
						if(distanceWithError<=radiusRange) {
							points_cloned.get(j).getAnchorsInRange().add(new Anchor(points_cloned.get(i), distanceWithError));
						}
					}
				}
				i=0;
				count++;
				ALE+=error;
			}
		}
		System.out.println("Iterative three closest: ALE: " + ALE/count + ", Localized nodes(%): " + (count*100)/getNonAnchorsSize() + "%");
	}
	
//	public List<Double> Trilateration(
//			Anchor awd1,
//			Anchor awd2,
//			Anchor awd3)
//	{
//        //DECLARE VARIABLES
//		List<Double> newCoordinates = new ArrayList<>();
//        double[] P1   = new double[2];
//        double[] P2   = new double[2];
//        double[] P3   = new double[2];
//        double[] ex   = new double[2];
//        double[] ey   = new double[2];
//        double[] p3p1 = new double[2];
//        double jval  = 0;
//        double temp  = 0;
//        double ival  = 0;
//        double p3p1i = 0;
//        double triptx;
//        double tripty;
//        double xval;
//        double yval;
//        double t1;
//        double t2;
//        double t3;
//        double t;
//        double exx;
//        double d;
//        double eyy;
//
//        //TRANSALTE POINTS TO VECTORS
//        //POINT 1
//        P1[0] = awd1.getP().getX();
//        P1[1] = awd1.getP().getY();
//        //POINT 2
//        P2[0] = awd2.getP().getX();
//        P2[1] = awd2.getP().getY();
//        //POINT 3
//        P3[0] = awd3.getP().getX();
//        P3[1] = awd3.getP().getY();
//
//        //TRANSFORM THE METERS VALUE FOR THE MAP UNIT
//        //DISTANCE BETWEEN POINT 1 AND MY LOCATION
//        double distance1 = (awd1.getDistance() / 100000);
//        //DISTANCE BETWEEN POINT 2 AND MY LOCATION
//        double distance2 = (awd2.getDistance() / 100000);
//        //DISTANCE BETWEEN POINT 3 AND MY LOCATION
//        double distance3 = (awd3.getDistance() / 100000);
//
//        for (int i = 0; i < P1.length; i++) {
//            t1   = P2[i];
//            t2   = P1[i];
//            t    = t1 - t2;
//            temp += (t*t);
//        }
//        d = Math.sqrt(temp);
//        for (int i = 0; i < P1.length; i++) {
//            t1    = P2[i];
//            t2    = P1[i];
//            exx   = (t1 - t2)/(Math.sqrt(temp));
//            ex[i] = exx;
//        }
//        for (int i = 0; i < P3.length; i++) {
//            t1      = P3[i];
//            t2      = P1[i];
//            t3      = t1 - t2;
//            p3p1[i] = t3;
//        }
//        for (int i = 0; i < ex.length; i++) {
//            t1 = ex[i];
//            t2 = p3p1[i];
//            ival += (t1*t2);
//        }
//        for (int  i = 0; i < P3.length; i++) {
//            t1 = P3[i];
//            t2 = P1[i];
//            t3 = ex[i] * ival;
//            t  = t1 - t2 -t3;
//            p3p1i += (t*t);
//        }
//        for (int i = 0; i < P3.length; i++) {
//            t1 = P3[i];
//            t2 = P1[i];
//            t3 = ex[i] * ival;
//            eyy = (t1 - t2 - t3)/Math.sqrt(p3p1i);
//            ey[i] = eyy;
//        }
//        for (int i = 0; i < ey.length; i++) {
//            t1 = ey[i];
//            t2 = p3p1[i];
//            jval += (t1*t2);
//        }
//        xval = (Math.pow(distance1, 2) - Math.pow(distance2, 2) + Math.pow(d, 2))/(2*d);
//        yval = ((Math.pow(distance1, 2) - Math.pow(distance3, 2) + Math.pow(ival, 2) + Math.pow(jval, 2))/(2*jval)) - ((ival/jval)*xval);
//
//        t1 = awd1.getP().getX();
//        t2 = ex[0] * xval;
//        t3 = ey[0] * yval;
//        triptx = t1 + t2 + t3;
//
//        t1 = awd1.getP().getY();
//        t2 = ex[1] * xval;
//        t3 = ey[1] * yval;
//        tripty = t1 + t2 + t3;
//
//
//        newCoordinates.add(triptx);
//        newCoordinates.add(tripty);
//        //return new LatLng(triptx,tripty);
//        return newCoordinates;
//
//    }
//	
	public List<Double> Trilateration(
			Anchor a1,
			Anchor a2,
			Anchor a3) {
		List<Double> newCoordinates = new ArrayList<>();
        double i1 = a1.getP().getX(), i2 = a2.getP().getX(),i3 = a3.getP().getX();
        double j1 = a1.getP().getY(), j2 = a2.getP().getY(),j3 = a3.getP().getY();
        double x, y;
        double d1 = a1.getDistance();
        double d2 = a2.getDistance();
        double d3 = a3.getDistance();

        x = (((((2 * j3) - (2 * j2)) * (((d1 * d1) - (d2 * d2)) + ((i2 * i2) - (i1 * i1)) + ((j2 * j2) - (j1 * j1)))) - (((2 * j2) - (2 * j1)) * (((d2 * d2) - (d3 * d3)) + ((i3 * i3) - (i2 * i2)) + ((j3 * j3) - (j2 * j2))))) / ((((2 * i2) - (2 * i3)) * ((2 * j2) - (2 * j1))) - (((2 * i1) - (2 * i2)) * ((2 * j3) - (2 * j2)))));
        y = (((d1 * d1) - (d2 * d2)) + ((i2 * i2) - (i1 * i1)) + ((j2 * j2) - (j1 * j1)) + (x * ((2 * i1) - (2 * i2)))) / ((2 * j2) - (2 * j1));

        newCoordinates.add(x);
        newCoordinates.add(y);
        return newCoordinates;
    }
	
	public static void main(String[]args) throws NumberFormatException, IOException {
		BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
		System.out.println("Enter field size: ");
		int size = Integer.parseInt(br.readLine());
		System.out.println("Enter the number of nodes: ");
		int numNodes = Integer.parseInt(br.readLine());
		System.out.println("Enter anchor nodes percentage:");
		int anchorPercent = Integer.parseInt(br.readLine());
		System.out.println("Enter the radius range of anchor nodes:");
		int radiusRange = Integer.parseInt(br.readLine());
		System.out.println("Enter chance of error in percentage:");
		int errorChancePercentage = Integer.parseInt(br.readLine());
		
		Field f = new Field(size,numNodes,anchorPercent,radiusRange, errorChancePercentage);
//		Field f = new Field(100,100,20,30,10);
		f.generateFieldAndPoints();
		f.nonIterative();
		f.iterativeMostRelevant();
//		f.iterativeThreeClosest();
	}
}
