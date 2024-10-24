// Implementation of plotter class
#include <simulator.h>
#include <plotter.h>
#include <matplot/matplot.h>
#include <memory>
#include <vector>

Plotter::Plotter(Aircraft& aircraft) : aircraft_(aircraft) 
{
        fig = std::make_shared<matplot::figure_type>();
        ax = fig->add_axes();
 }


void Plotter::updatePlot(double x, double y) 
{
    // Add the new point to the data vectors
    xData.push_back(x);
    yData.push_back(y);
    // Plot the updated data
        // If this is the first plot, create a new plot and store the handle
    if (!line_handle) 
    {
        // Create the plot for the first time and store the line handle
        line_handle = ax->plot(xData, yData);
    } 
    else
    {
        // Update the existing plot with new data (more efficient)
        line_handle->x_data(xData);
        line_handle->y_data(yData);
    }

    // Set x-axis range from 0 to 1000
    ax->xlim({0, 10000});
    ax->ylim({0, 40});
    ax->xlabel("Steps");
    ax->ylabel("V [m/s]");
    //ax->line_width(3); // sets width of the borders
    //ax->x_grid(true);

    //line_handle->line_width(2);
    
        
        
        
        fig->draw(); // Draw the current figure using the figure instance
}



