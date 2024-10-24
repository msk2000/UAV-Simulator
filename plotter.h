// Header file for plotter class
#ifndef PLOTTER_H
#define PLOTTER_H

#include <matplot/matplot.h>
#include <vector>

class Plotter 
{
public:
    Plotter(Aircraft& aircraft);
    void updatePlot(double x, double y);
    std::vector<double> xData;  // Data to plot on the x-axis
    std::vector<double> yData;  // Data to plot on the y-axis
    // Initialize the plot handle as nullptr initially
    matplot::line_handle line_handle;  // Handle to the line plot
    


private:
    matplot::figure_handle fig; // Handle for figure
    matplot::axes_handle ax;    // Handle for axes
    
    Aircraft& aircraft_;        // Reference to the Aircraft object for data access
};

#endif // PLOTTER_H
