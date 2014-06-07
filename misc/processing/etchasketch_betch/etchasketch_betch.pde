/** etchasketch_betch
* You, like, totally use your mouse to, uh
* draw a picture and liek stuff. Press the
* space bar to save your ugly drawing. Press
* the party button to have liek a totally
* awesome party. Where is like the party button
* betch? Check ur phone -- under my name.

% TODO %
* Remove duplicate array entries to save space and time
* Get the right scaling factor by touching real life etch-a-sketch
* Shooz
*/

// Global variables!
float SCALE_FACTOR = 1.0;  // This needs to be tuned for the actual robit
IntList xvals, yvals;
int STARTX;
int STARTY;
 

void setup(){
  size(2*250, 2*1300/8);  // Establishes screen size
  STARTX = 0;           // Pixels from left
  STARTY = 0;           // Pixels from top
  //noSmooth();         // noSmooth() aliases the line. I like it smooth.
  strokeWeight(2);      // Line thickness
  
  // Storage space for mouse values.
  // Declares two new intlists, instantiates them,
  // and creates the first element at (0,0)
  xvals = new IntList(); 
  yvals = new IntList();  
  xvals.append(STARTX);
  yvals.append(STARTY);
}

void draw(){
  // Paint it Etch-a-Sketch gray
  background(216,216,216);
  
  // Redraw shape
  for(int i = 1; i < xvals.size(); i++){
    line(xvals.get(i-1), yvals.get(i-1), xvals.get(i), yvals.get(i));
  }
  
  // Only add more to the shape if the mouse button is pressed!
  if(mousePressed){
    fill(0); 
    xvals.append(mouseX);
    yvals.append(mouseY);
  }
}

void keyPressed(){
  if (key == ' '){
    // Make a string array!
    String[] coordinates_lin = new String[xvals.size()];
    String[] coordinates_rad = new String[xvals.size()];
    
    // Make file names!
    String filename_lin = "etchasketch_betch_lin_"+str(year())+str(month())+str(day())+str(hour())+str(minute())+str(second())+".txt";
    String filename_rad = "etchasketch_betch_rad_"+str(year())+str(month())+str(day())+str(hour())+str(minute())+str(second())+".txt";
    
    // Flash the screen!
    background(0,0,0);
    
    // Prep the values to be written to file!
    // We assume clockwise rotation increases x and y (which makes x go right and y go down)
    for(int i = 0; i < xvals.size(); i++){
      coordinates_lin[i] = str(xvals.get(i)) + ',' + str(yvals.get(i));
      if(i>0){
        coordinates_rad[i-1] = str(float(xvals.get(i) - xvals.get(i-1)) / SCALE_FACTOR) + ',' + str(float(yvals.get(i) - yvals.get(i-1)) / SCALE_FACTOR);
      }
    }
 
    // Writes the strings to a file! Each on a separate line!!
    saveStrings(filename_lin, coordinates_lin);
    saveStrings(filename_rad, coordinates_rad);
    
    // Clear the variables!
    xvals.clear(); 
    yvals.clear();  
    xvals.append(STARTX);
    yvals.append(STARTY);
  }
}
