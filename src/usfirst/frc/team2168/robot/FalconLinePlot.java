package usfirst.frc.team2168.robot;

import java.awt.*;
import java.awt.datatransfer.Clipboard;
import java.awt.datatransfer.ClipboardOwner;
import java.awt.datatransfer.DataFlavor;
import java.awt.datatransfer.Transferable;
import java.awt.datatransfer.UnsupportedFlavorException;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.*;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.LinkedList;

import javax.swing.*;

/**
 * This class is a basic plotting class using the Java AWT interface. It has basic features which allow the user 
 * to plot multiple graphs on one figure, control axis dimensions, and specify colors.
 * 
 * This is by all means not an extensive plotter, but it will help visualize data very quickly and accurately. If
 * a more robust plotting function is required, the user is encouraged to use Excel or Matlab. The purpose of this
 * class is to be easy to use with enought automation to have nice graphs with minimal effort, but give the user
 * control over as much as possible, so they can generate the perfect chart.
 * 
 * The plotter also features the ability to capture screen shots directly from the right-click menu, this allows
 * the user to copy and paste plots into reports or other documents rather quickly.
 * 
 * This class holds an interface similar to that of Matlab. 
 * 
 * This class currently only supports scatterd line charts.
 * 
 * @author Kevin Harrilal
 * @email kevin@team2168.org
 * @version 1
 * @date 9 Sept 2014
 *
 */
 
class FalconLinePlot extends JPanel implements ClipboardOwner{
  
    
	private static final long serialVersionUID = 3205256608145459434L;
	private final int yPAD = 60; //controls how far the X- and Y- axis lines are away from the window edge
    private final int xPAD = 70; //controls how far the X- and Y- axis lines are away from the window edge
    
    private double upperXtic;
    private double lowerXtic;
    private double upperYtic;
    private double lowerYtic;
    private boolean yGrid;
    private boolean xGrid;
    
    private double yMax;
    private double yMin;
    private double xMax;
    private double xMin;
    
    private int yticCount;
    private int xticCount;
    private double xTicStepSize;
    private double yTicStepSize;
    
    boolean userSetYTic;
    boolean userSetXTic;
    
    private String xLabel;
    private String yLabel;
    private String titleLabel;
    protected static int count = 0;
    
    JPopupMenu menu = new JPopupMenu("Popup");
    
    //Link List to hold all different plots on one graph.
    private LinkedList<xyNode> link; 
    

    /**
     * Constructor which Plots only Y-axis data.
     * @param yData is a array of doubles representing the Y-axis values of the data to be plotted.
     */
    public FalconLinePlot(double[] yData)
    {
    	this(null,yData,Color.red);
    }
    
    public FalconLinePlot(double[] yData,Color lineColor, Color marker)
    {
    	this(null,yData,lineColor,marker);
    }
    
    /**
     * Constructor which Plots chart based on provided x and y data. X and Y arrays must be of the same length.
     * @param xData is an array of doubles representing the X-axis values of the data to be plotted.
     * @param yData is an array of double representing the Y-axis values of the data to be plotted.
     */
    public FalconLinePlot(double[] xData, double[] yData)
    {
    	this(xData,yData,Color.red,null);
    }
    
    /**
     * Constructor which Plots chart based on provided x and y axis data. 
     * @param data is a 2D array of doubles of size Nx2 or 2xN. The plot assumes X is the first dimension, and y data
     * is the second dimension.
     */
    public FalconLinePlot(double[][] data)
    {
    	this(getXVector(data),getYVector(data),Color.red,null);
    }
    
/**
 * Constructor which plots charts based on provided x and y axis data in a single two dimensional array.
 * @param data is a 2D array of doubles of size Nx2 or 2xN. The plot assumes X is the first dimension, and y data
 * is the second dimension.
 * @param lineColor is the color the user wishes to be displayed for the line connecting each datapoint
 * @param markerColor is the color the user which to be used for the data point. Make this null if the user wishes to
 * not have datapoint markers.
 */
    public FalconLinePlot(double[][] data, Color lineColor, Color markerColor)
    {
    	this(getXVector(data),getYVector(data),lineColor,markerColor);
    }
    
    /**
     * Constructor which plots charts based on provided x and y axis data provided as separate arrays. The user can also specify the color of the adjoining line.
     * Data markers are not displayed.
     * @param xData is an array of doubles representing the X-axis values of the data to be plotted.
     * @param yData is an array of double representing the Y-axis values of the data to be plotted.
     * @param lineColor is the color the user wishes to be displayed for the line connecting each datapoint
     */
    public FalconLinePlot(double[] xData, double[] yData,Color lineColor)
    {
    	this(xData,yData,lineColor,null);
    }
    

    
    /**
     * Constructor which plots charts based on provided x and y axis data, provided as separate arrays. The user 
     * can also specify the color of the adjoining line and the color of the datapoint maker.
     * @param xData is an array of doubles representing the X-axis values of the data to be plotted.
     * @param yData is an array of double representing the Y-axis values of the data to be plotted.
     * @param lineColor is the color the user wishes to be displayed for the line connecting each datapoint
     * @param markerColor is the color the user which to be used for the data point. Make this null if the user wishes to
     * not have datapoint markers.
     */
    public FalconLinePlot(double[] xData, double[] yData,Color lineColor, Color markerColor)
    {
    	xLabel = "X axis";
    	yLabel = "Y axis";
    	titleLabel = "Title";
    	
       upperXtic = -Double.MAX_VALUE;
       lowerXtic = Double.MAX_VALUE;
       upperYtic = -Double.MAX_VALUE;
       lowerYtic = Double.MAX_VALUE;
       xticCount = -Integer.MAX_VALUE;
       
       this.userSetXTic = false;
       this.userSetYTic = false;
    	
    	link = new LinkedList<xyNode>();
    	
    	addData(xData, yData, lineColor,markerColor);
    	
    	count ++;
    	JFrame g = new JFrame("Figure " + count);
        g.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        g.add(this);
        g.setSize(600,400);
        g.setLocationByPlatform(true);
        g.setVisible(true);
         
        menu(g,this);	
    }
    
    /**
     * Adds a plot to an existing figure.  
     * @param yData is a array of doubles representing the Y-axis values of the data to be plotted.
     * @param color is the color the user wishes to be displayed for the line connecting each datapoint
     */
    
    public void addData(double[] y, Color lineColor)
    {
    	addData(y, lineColor, null);
    }
    
    public void addData(double[] y, Color lineColor, Color marker)
    {
    	//cant add y only data unless all other data is y only data
    	for(xyNode data: link)
    		if(data.x != null)
    			throw new Error ("All previous chart series need to have only Y data arrays");
    	
    	addData(null,y,lineColor, marker);
    }
    
    public void addData(double[] x, double[] y, Color lineColor)
    {
    	addData(x,y,lineColor,null);
    }
    
    
    public void addData(double[][] data, Color lineColor)
    {
    	addData(getXVector(data),getYVector(data),lineColor,null);
    }
    
    public void addData(double[][] data, Color lineColor, Color marker)
    {
    	addData(getXVector(data),getYVector(data),lineColor,marker);
    }
    
    public void addData(double[] x, double[] y, Color lineColor, Color marker)
    { 	
    	xyNode Data = new xyNode();
    	    	
    	//copy y array into node
    	Data.y = new double[y.length];
    	Data.lineColor = lineColor;
    	
    	if(marker == null)
    		Data.lineMarker = false;
    	else
    	{
    		Data.lineMarker = true;
    		Data.markerColor = marker;
    	}
    	for(int i=0; i<y.length; i++)
    		Data.y[i] = y[i];
    	
    	//if X is not null, copy x
    	if(x != null)
    	{
        	//cant add x, and y data unless all other data has x and y data
        	
        	for(xyNode data: link)
        		if(data.x == null)
        			throw new Error ("All previous chart series need to have both X and Y data arrays");
    		
    		if(x.length != y.length)
    			throw new Error("X dimension must match Y dimension");
    		
    		Data.x = new double[x.length];
    		
    		for(int i=0; i<x.length; i++)
        		Data.x[i] = x[i];
    		
    	}
    	link.add(Data);
    }
 
    /**
     * Main method which paints the panel and shows the figure.
     * 
     */
    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 =  (Graphics2D)g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        
        int w = getWidth();
        int h = getHeight();
    
        // Draw X and Y lines axis.
        Line2D yaxis = new Line2D.Double(xPAD, yPAD, xPAD, h-yPAD);
        Line2D.Double xaxis = new Line2D.Double(xPAD, h-yPAD, w-xPAD, h-yPAD);
        g2.draw(yaxis); 
        g2.draw(xaxis);
        
        //find Max Y limits
        getMinMax(link);
        
        //draw ticks
        drawYTickRange(g2, yaxis, 15, yMax, yMin);
        drawXTickRange(g2, xaxis, 15, xMax, xMin);
        
        //plot all data
        plot(g2);
        
        //draw x and y labels
        setXLabel(g2, xLabel);
        setYLabel(g2, yLabel);
        setTitle(g2, titleLabel);
        
    }
    
    void setXTic(double lowerBound, double upperBound, double stepSize)
    {
    	this.userSetXTic = true;
    	
    	this.upperXtic=upperBound;
    	this.lowerXtic=lowerBound;
    	this.xTicStepSize = stepSize;
    }
    
    public void setYTic(double lowerBound, double upperBound, double stepSize)
    {
    	this.userSetYTic = true;
    	
    	this.upperYtic=upperBound;
    	this.lowerYtic=lowerBound;
    	this.yTicStepSize = stepSize;
    }
    
    private void plot(Graphics2D g2)
    {
    	
    	int w = super.getWidth();
    	int h = super.getHeight();
    	
    	Color tempC = g2.getColor();
    	
    	//loop through list and plot each
    	for(int i=0; i<link.size(); i++)
    	{
	        // Draw lines.
	        double xScale = (double)(w - 2*xPAD)/(upperXtic-lowerXtic);
	        double yScale = (double)(h - 2*yPAD)/(upperYtic-lowerYtic);
	        
	        for(int j = 0; j < link.get(i).y.length-1; j++) 
	        {
	        	double x1;
	        	double x2;
	        	
	            if(link.get(i).x==null)
	            {
	            	x1 = xPAD + j*xScale;
	            	x2 = xPAD + (j+1)*xScale;
	            }
	            else
	            {
	            	x1 = xPAD + xScale*link.get(i).x[j] + lowerXtic*xScale;
	            	x2 = xPAD + xScale*link.get(i).x[j+1] + lowerXtic*xScale;
	            }
	            
	            double y1 = h - yPAD - yScale*link.get(i).y[j] + lowerYtic*yScale;
	            
	            double y2 = h - yPAD - yScale*link.get(i).y[j+1] + lowerYtic*yScale;
	            g2.setPaint(link.get(i).lineColor);
	            g2.draw(new Line2D.Double(x1, y1, x2, y2));
	            
	            if(link.get(i).lineMarker)
	            {
	            	g2.setPaint(link.get(i).markerColor);
	            	g2.fill(new Ellipse2D.Double(x1-2, y1-2, 4, 4));
	            	g2.fill(new Ellipse2D.Double(x2-2, y2-2, 4, 4));
	            }
	      
	        }
	        	        	
    	}
    	
    	g2.setColor(tempC);
    }
    
    /**
     * need to optimize for loops
     * @param list
     */
    private void getMinMax(LinkedList<xyNode> list)
    {
    	for(xyNode node: list)
    	{
    		double tempYMax = getMax(node.y);
    		double tempYMin = getMin(node.y);
    		
    		if(tempYMin<yMin)
    			yMin = tempYMin;
    		
    		if(tempYMax>yMax)
    			yMax=tempYMax;
    		
    		if(xticCount < node.y.length)
				xticCount = node.y.length;
    		
    		
    		if(node.x != null)
    		{
        		double tempXMax = getMax(node.x);
        		double tempXMin = getMin(node.x);
        		
        		if(tempXMin<xMin)
        			xMin = tempXMin;
        		
        		if(tempXMax>xMax)
        			xMax=tempXMax;
        		
    		}
    		else
    		{
    			xMax=node.y.length-1;
    			xMin=0;
    			
    		}
    		
    	}
    	
    }
 
    private double getMax(double[] data) {
        double max = -Double.MAX_VALUE;
        for(int i = 0; i < data.length; i++) {
            if(data[i] > max)
                max = data[i];
        }
        return max;
    }
    
    private double getMin(double[] data) {
        double min = Double.MAX_VALUE;
        for(int i = 0; i < data.length; i++) {
            if(data[i] < min)
                min = data[i];
        }
        return min;
    }
    
    public void setYLabel(String s)
    {
    	yLabel = s;
    }
    
    public void setXLabel(String s)
    {
    	xLabel = s;
    }
    
    public void setTitle(String s)
    {
    	titleLabel = s;
    }
    
    private void setYLabel(Graphics2D g2, String s)
    {
    	FontMetrics fm = getFontMetrics(getFont());
    	int width = fm.stringWidth(s);
    	
    	AffineTransform temp = g2.getTransform();
    	
    	    AffineTransform at = new AffineTransform();
    	    at.setToRotation(-Math.PI /2, 10, getHeight()/2+width/2);
    	    g2.setTransform(at);
    	    
    	    //draw string in center of y axis
    	    g2.drawString(s, 10, 7+getHeight()/2+width/2);

        g2.setTransform(temp);
    	
    }
    
    private void setXLabel(Graphics2D g2, String s)
    {
    	FontMetrics fm = getFontMetrics(getFont());
    	int width = fm.stringWidth(s);
    	
    	g2.drawString(s, getWidth()/2-(width/2), getHeight()-10);
    }
    
    private void setTitle(Graphics2D g2, String s)
    {
    	FontMetrics fm = getFontMetrics(getFont());
    	
    	String[] line = s.split("\n");
    	
    	int height = xPAD/2 - ((line.length-1) * fm.getHeight()/2);
    	
    	for (int i=0; i<line.length; i++)
    	{
    		
    		int width = fm.stringWidth(line[i]);
            g2.drawString(line[i], getWidth()/2-(width/2),  height);
            height +=fm.getHeight();
            
    	}
   
    }

    
    public void yGridOn()
    {
    	yGrid=true;
    	//super.repaint();
    }
    
    public void yGridOff()
    {
    	yGrid=false;
    	//super.repaint();
    }
    
    public void xGridOn()
    {
    	xGrid=true;
    	//super.repaint();
    }
    
    public void xGridOff()
    {
    	xGrid=false;
    	//super.repaint();
    }
    
    private void drawYTickRange(Graphics2D g2, Line2D yaxis, int tickCount, double Max, double Min)
    {
    	if(!userSetYTic)
    	{
    	double range = Max - Min;
    	
    	
    	//calculate max Y and min Y tic Range
    	double unroundedTickSize = range/(tickCount-1);
    	double x = Math.ceil(Math.log10(unroundedTickSize)-1);
    	double pow10x = Math.pow(10, x);
    	yTicStepSize = Math.ceil(unroundedTickSize / pow10x) * pow10x;
    	
    	//determine min and max tick label 
        if(Min<0)
        	lowerYtic = yTicStepSize * Math.floor(Min/yTicStepSize);
        else
        	lowerYtic = yTicStepSize * Math.ceil(Min/yTicStepSize);
        
        if(Max<0)
        	upperYtic = yTicStepSize * Math.floor(1+Max/yTicStepSize);
        else
        	upperYtic = yTicStepSize * Math.ceil(1+Max/yTicStepSize);
        
    	}
    	
        
        double x0 = yaxis.getX1();
        double y0 = yaxis.getY1();
        double xf = yaxis.getX2();
        double yf = yaxis.getY2();
        
        //calculate stepsize between ticks and length of Y axis using distance formula
        int roundedTicks = (int) ((upperYtic - lowerYtic) / yTicStepSize);
        double distance = Math.sqrt(Math.pow((xf-x0), 2)+Math.pow((yf-y0), 2)) / roundedTicks;
        
        double upper = upperYtic;
        for (int i = 0; i<=roundedTicks; i++)
        {
        	double newY = y0;
        	
        	//calculate width of number for proper drawing
        	String number = new DecimalFormat("#.#").format(upper);
        	FontMetrics fm = getFontMetrics(getFont());
        	int width = fm.stringWidth(number);
   
        	g2.draw(new Line2D.Double(x0,newY, x0-10,newY));
        	g2.drawString(number, (float)x0-15-width, (float)newY+5); 
        	 
        	//add grid lines to chart
        	if(yGrid && i!=roundedTicks)
        	{

        		Stroke tempS = g2.getStroke();
        		Color tempC = g2.getColor();
        		
        		g2.setColor (Color.lightGray);
          	    g2.setStroke (new BasicStroke(
          	      1f, 
          	      BasicStroke.CAP_ROUND, 
          	      BasicStroke.JOIN_ROUND, 
          	      1f, 
          	      new float[] {5f}, 
          	      0f));
          	    
          	    g2.draw(new Line2D.Double(xPAD, newY, getWidth()-xPAD, newY));
        		
          	    g2.setColor(tempC);
          	    g2.setStroke(tempS);
        		
        	}
        	
        	  upper = upper - yTicStepSize;
        	y0 = newY + distance;
        	
        }
    }
    
    private void drawXTickRange(Graphics2D g2, Line2D xaxis, int tickCount, double Max, double Min)
    {
    	drawXTickRange(g2, xaxis, tickCount, Max, Min, 1);
    }
    
    private void drawXTickRange(Graphics2D g2, Line2D xaxis, int tickCount, double Max, double Min, double skip)
    {
    	if(!userSetXTic)
    	{
    	double range = Max - Min;
    	
    	//calculate max Y and min Y tic Range
    	double unroundedTickSize = range/(tickCount-1);
    	double x = Math.ceil(Math.log10(unroundedTickSize)-1);
    	double pow10x = Math.pow(10, x);
    	xTicStepSize = Math.ceil(unroundedTickSize / pow10x) * pow10x;
    	
    	//determine min and max tick label 
        if(Min<0)
        	lowerXtic = xTicStepSize * Math.floor(Min/xTicStepSize);
        else
        	lowerXtic = xTicStepSize * Math.ceil(Min/xTicStepSize);
        
        if(Max<0)
        	upperXtic = xTicStepSize * Math.floor(1+Max/xTicStepSize);
        else
        	upperXtic = xTicStepSize * Math.ceil(1+Max/xTicStepSize);
    	}
        
        
        double x0 = xaxis.getX1();
        double y0 = xaxis.getY1();
        double xf = xaxis.getX2();
        double yf = xaxis.getY2();
        
        //calculate stepsize between ticks and length of Y axis using distance formula
        int roundedTicks = (int) ((upperXtic - lowerXtic) / xTicStepSize);
        
        
        double distance = Math.sqrt(Math.pow((xf-x0), 2)+Math.pow((yf-y0), 2)) / roundedTicks;
        
        double lower = lowerXtic;
        for (int i = 0; i<=roundedTicks; i++)
        {
        	double newX = x0;
        	
        	//calculate width of number for proper drawing
        	String number = new DecimalFormat("#.#").format(lower);
        	FontMetrics fm = getFontMetrics( getFont() );
        	int width = fm.stringWidth(number);
        	
   
        	g2.draw(new Line2D.Double(newX,yf, newX,yf+10));
        	
        	//dont label every x tic to prevent clutter
        	if(i%skip==0)
       		g2.drawString(number, (float)(newX-(width/2.0)), (float)yf+25); 
        	
        	//add grid lines to chart
        	if(xGrid && i!=0)
        	{
        		Stroke tempS = g2.getStroke();
        		Color tempC = g2.getColor();
        		
        		g2.setColor (Color.lightGray);
          	    g2.setStroke (new BasicStroke(
          	      1f, 
          	      BasicStroke.CAP_ROUND, 
          	      BasicStroke.JOIN_ROUND, 
          	      1f, 
          	      new float[] {5f}, 
          	      0f));
          	    
          	    g2.draw(new Line2D.Double(newX, yPAD, newX, getHeight()-yPAD));
        		
          	    g2.setColor(tempC);
          	    g2.setStroke(tempS);
        		
        	}
        	
        	 
        	lower = lower + xTicStepSize;
        	x0 = newX + distance;
        }
    }
    
    public void updateData(int series, double[][] data)
    {
    	//add Data to link list
    	addData(data,null,null);
    	
    	//copy data from new to old and line styles from list to new list.
    	
    	link.get(series).x = link.getLast().x.clone();
    	link.get(series).y = link.getLast().y.clone();
    	
    	//remove last data
    	link.removeLast();
    
    	
    	
    }
    
	public static double[] getXVector(double[][] arr)
	{
		double[] temp = new double[arr.length];

		for(int i=0; i<temp.length; i++)
			temp[i] = arr[i][0];

		return temp;		
	}

	public static double[] getYVector(double[][] arr)
	{
		double[] temp = new double[arr.length];

		for(int i=0; i<temp.length; i++)
			temp[i] = arr[i][1];

		return temp;		
	}
	
	/**********Class for Linked List************/
    private class xyNode
    {
    	double[] x;
    	double[] y;
    	Color lineColor;
    	
    	boolean lineMarker;
    	Color markerColor;
    	
    	public xyNode()
    	{
    		x=null;
    		y=null;
    		
    		lineMarker = false;
    	}
    }
	
    /****Methods to Support Right Click Menu****/
    @Override
	public void lostOwnership(Clipboard    clip, Transferable transferable) 
	{
		//We must keep the object we placed on the system clipboard
		//until this method is called.
	}

    
    
    private void menu(JFrame g, final FalconLinePlot p )
    {

    	g.addMouseListener(new PopupTriggerListener());
    	
    	JMenuItem item = new JMenuItem("Copy Figure");

        item.addActionListener(new ActionListener() 
        {
          public void actionPerformed(ActionEvent e) 
          {
            
            BufferedImage i = new BufferedImage(p.getSize().width, p.getSize().height,BufferedImage.TRANSLUCENT);
            p.setOpaque(false);
            p.paint(i.createGraphics()); 
            TransferableImage trans = new TransferableImage( i );
			Clipboard c = Toolkit.getDefaultToolkit().getSystemClipboard();
			c.setContents( trans, p);
          }
        });
        
        menu.add(item);
    	
        item = new JMenuItem("Desktop ScreenShot");
        item.addActionListener(new ActionListener() 
        {
          public void actionPerformed(ActionEvent e) 
          {
            System.out.println("Copy files to clipboard");
            
            try {
                Robot robot = new Robot();
                Dimension screenSize  = Toolkit.getDefaultToolkit().getScreenSize();
                Rectangle screen = new Rectangle( screenSize );
                BufferedImage i = robot.createScreenCapture( screen );
                TransferableImage trans = new TransferableImage( i );
                Clipboard c = Toolkit.getDefaultToolkit().getSystemClipboard();
                c.setContents( trans, p);
            }
            catch ( AWTException x ) {
                x.printStackTrace();
                System.exit( 1 );
            }
            
          }
        });
        
        menu.add(item);
    }

    class PopupTriggerListener extends MouseAdapter {
    	public void mousePressed(MouseEvent ev) {
    		if (ev.isPopupTrigger()) {
    			menu.show(ev.getComponent(), ev.getX(), ev.getY());
    		}
    	}

    	public void mouseReleased(MouseEvent ev) {
    		if (ev.isPopupTrigger()) {
    			menu.show(ev.getComponent(), ev.getX(), ev.getY());
    		}
    	}

    	public void mouseClicked(MouseEvent ev) {

    	}
    }

    private class TransferableImage implements Transferable {

        Image i;

        public TransferableImage( Image i ) {
            this.i = i;
        }

        public Object getTransferData( DataFlavor flavor )
        throws UnsupportedFlavorException, IOException {
            if ( flavor.equals( DataFlavor.imageFlavor ) && i != null ) {
                return i;
            }
            else {
                throw new UnsupportedFlavorException( flavor );
            }
        }

        public DataFlavor[] getTransferDataFlavors() {
            DataFlavor[] flavors = new DataFlavor[ 1 ];
            flavors[ 0 ] = DataFlavor.imageFlavor;
            return flavors;
        }

        public boolean isDataFlavorSupported( DataFlavor flavor ) {
            DataFlavor[] flavors = getTransferDataFlavors();
            for ( int i = 0; i < flavors.length; i++ ) {
                if ( flavor.equals( flavors[ i ] ) ) {
                    return true;
                }
            }

            return false;
        }
    }
    
    /******TEST MAIN METHOD*******/
    public static void main(String[] args) {
    	
  	  double[] data = {
  		        -235, 14, 18, 03, 60, 150, 74, 87, 54, 77,
  		        61, 55, 48, 60, 49, 36, 38, 27, 20, 18,5
  		    };
  	  
  	  double[] data2 = {
		        -4, 124, 128, 33, -1, 1, 74, 87, 54, 77,
		        61, 55, 48, 60, 40, 36, 38, 27, 20, 18,5
		    };
  	  
  	  double[] test = {0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 2.0,
  			  2.2,2.4,2.6,2.8,3.0,3.2,3.4,3.6,3.8,4.0,4.2
  	  };
  	
  	
  	
  	FalconLinePlot fig2 = new FalconLinePlot(data,Color.red, Color.blue);
  	
  	fig2.yGridOn();
  	fig2.xGridOn();
  	fig2.setYLabel("This is a new");
  	
  	fig2.addData(data2, Color.blue);
  	
  	FalconLinePlot fig1 =  new FalconLinePlot(test,data);
  	
  }
    
    
}
   
