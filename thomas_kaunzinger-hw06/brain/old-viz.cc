
// Cairo Drawing Window
// Based on Gnome doc example.
// ref: https://developer.gnome.org/gtk3/stable/ch01s05.html

#include <iostream>
#include <thread>
#include <mutex>
#include <cmath>
using namespace std;

typedef lock_guard<mutex> guard;

#include <X11/Xlib.h>
#include <gtk/gtk.h>
#include "viz.hh"

/* Surface to store current scribbles */
std::mutex mx;
static cairo_surface_t* surface = NULL;
static GtkWidget *window = NULL;
static GtkWidget *drawing_area = NULL;

static void
clear_surface (void)
{
    cairo_t *cr;

    cr = cairo_create(surface);

    cairo_set_source_rgb(cr, 1, 1, 1);
    cairo_paint(cr);

    cairo_destroy(cr);
}

/* Create a new surface of the appropriate size to store our scribbles */
static gboolean
configure_event_cb(GtkWidget         *widget,
                   GdkEventConfigure *event,
                   gpointer           data)
{
    guard _gg(mx);
    if (surface) {
        cairo_surface_destroy(surface);
    }

    surface = gdk_window_create_similar_surface(
        gtk_widget_get_window(widget),
        CAIRO_CONTENT_COLOR,
        gtk_widget_get_allocated_width(widget),
        gtk_widget_get_allocated_height(widget)
    );

    /* Initialize the surface to white */
    clear_surface();

    /* We've handled the configure event, no need for further processing. */
    return TRUE;
}

/* Redraw the screen from the surface. Note that the ::draw
 * signal receives a ready-to-be-used cairo_t that is already
 * clipped to only draw the exposed areas of the widget
 */
static gboolean
draw_cb(GtkWidget *widget,
        cairo_t   *cr,
        gpointer   data)
{
    guard _gg(mx);
    cairo_set_source_surface(cr, surface, 0, 0);
    cairo_paint(cr);

    /*
    int ww, hh;
    gtk_window_get_size(GTK_WINDOW(window), &ww, &hh);
    cout << "window: " << ww << "," << hh << endl;
    */

    return FALSE;
}

/* Draw a rectangle on the surface at the given position */
// MODIFIED BY THOMAS TO SUPPORT COLOR
static void
draw_brush(GtkWidget *widget,
           gdouble    x,
           gdouble    y,
           float r,
           float g,
           float b)
{
    cairo_t *cr;

    //GdkRectangle rect;
    //gtk_widget_get_allocation(widget, &rect);
    //cout << "rect: " << rect.x << "," << rect.y << endl;

    /* Paint to the surface, where we store our state */
    cr = cairo_create(surface);

    cairo_set_source_rgb(cr, r, g, b);
    cairo_rectangle(cr, x - 3, y - 3, 6, 6);
    cairo_fill(cr);

    cairo_destroy(cr);

    /* Now invalidate the affected region of the drawing area. */
    gtk_widget_queue_draw_area(widget, x - 3, y - 3, 6, 6);
}

/* Handle button press events by either drawing a rectangle
 * or clearing the surface, depending on which button was pressed.
 * The ::button-press signal handler receives a GdkEventButton
 * struct which contains this information.
 */
static gboolean
button_press_event_cb(GtkWidget      *widget,
                      GdkEventButton *event,
                      gpointer        data)
{
    cout << "click" << endl;

    guard _gg(mx);
    /* paranoia check, in case we haven't gotten a configure event */
    if (surface == NULL)
        return FALSE;

    /*
    if (event->button == GDK_BUTTON_PRIMARY)
    {
        draw_brush(widget, event->x, event->y);
    }
    else 
    */
      
    if (event->button == GDK_BUTTON_SECONDARY)
    {
        clear_surface();
        gtk_widget_queue_draw(widget);
    }

    /* We've handled the event, stop processing */
    return TRUE;
}

/* Handle motion events by continuing to draw if button 1 is
 * still held down. The ::motion-notify signal handler receives
 * a GdkEventMotion struct which contains this information.
 */
static gboolean
motion_notify_event_cb (GtkWidget      *widget,
                        GdkEventMotion *event,
                        gpointer        data)
{
    guard _gg(mx);
    /* paranoia check, in case we haven't gotten a configure event */
    if (surface == NULL)
        return FALSE;

    /*
      if (event->state & GDK_BUTTON1_MASK)
      draw_brush (widget, event->x, event->y);
    */

    /* We've handled it, stop processing */
    return TRUE;
}

static void
close_window (void)
{
    guard _gg(mx);
    if (surface) {
        cairo_surface_destroy(surface);
    }
}

static void
activate (GtkApplication *app,
          gpointer        user_data)
{
    GtkWidget *frame;

    window = gtk_application_window_new(app);
    gtk_window_set_title(GTK_WINDOW(window), "Viz!");

    g_signal_connect(window, "destroy", G_CALLBACK(close_window), NULL);

    gtk_container_set_border_width(GTK_CONTAINER(window), 8);

    frame = gtk_frame_new(NULL);
    gtk_frame_set_shadow_type(GTK_FRAME(frame), GTK_SHADOW_IN);
    gtk_container_add(GTK_CONTAINER(window), frame);

    drawing_area = gtk_drawing_area_new();
    /* set a minimum size */
    gtk_widget_set_size_request(drawing_area, 600, 600);

    gtk_container_add(GTK_CONTAINER(frame), drawing_area);

    /* Signals used to handle the backing surface */
    g_signal_connect(drawing_area, "draw",
                     G_CALLBACK(draw_cb), NULL);
    g_signal_connect(drawing_area,"configure-event",
                     G_CALLBACK(configure_event_cb), NULL);

    /* Event signals */
    g_signal_connect (drawing_area, "motion-notify-event",
                      G_CALLBACK (motion_notify_event_cb), NULL);
    g_signal_connect(drawing_area, "button-press-event",
                     G_CALLBACK(button_press_event_cb), NULL);

    /* Ask to receive events the drawing area doesn't normally
     * subscribe to. In particular, we need to ask for the
     * button press and motion notify events that want to handle.
     */
    gtk_widget_set_events(drawing_area, gtk_widget_get_events (drawing_area)
                          | GDK_BUTTON_PRESS_MASK
                          | GDK_POINTER_MOTION_MASK);

    gtk_widget_show_all(window);
    //gdk_threads_init();
}


//////////////////////////////
// ACTUAL MAP UPDATING!!!!! //
//////////////////////////////

// Imports
#include <unordered_map>
#include <vector>

// Math macros
#define d2r(degrees) ((degrees) * M_PI / 180.0)
#define r2d(radians) ((radians) * 180.0 / M_PI)
#define clamp(min, x, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

// Clogged up namespace smh
using namespace std;

// Constants
#define k_width_meters      70
#define k_height_meters     70
#define k_granularity       0.25

// Mutable variables

// Function instantiation
float diff(float deg1, float deg2);
vector<coord> bresenham(coord pos1, coord pos2);

// Something is hit -> update map
void viz_hit(unordered_map<coord, cell_params> occupancy_grid,
            float robot_x, float robot_y, float robot_t,
            float range, float angle) {
    
    //GDK_THREADS_ENTER();
    guard _gg(mx);
    
    struct coord robot_pos;
    robot_pos.x = static_cast<int>(round(robot_x / k_granularity));
    robot_pos.y = static_cast<int>(round(robot_y / k_granularity));
    float current_angle = r2d(robot_t);

    float actual_angle = diff(r2d(angle), current_angle);
    float relative_x = 0;
    float relative_y = 0;

    //             90   +y
    //                |
    //                |   
    //              2 | 1
    // +/-180 --------|-------- 0
    //   -x         3 | 4       +x
    //                |    
    //                |
    //            -90   -y

    // 1
    if (actual_angle <= 0 && actual_angle > -90) {
        relative_x = cos(d2r(actual_angle)) * range;
        relative_y = sin(d2r(actual_angle)) * range;
    }
    // 2
    else if (actual_angle > 90 && actual_angle <= 180) {
        relative_x = -1 * cos(d2r(180 - actual_angle)) * range;
        relative_y =      sin(d2r(180 - actual_angle)) * range;
    }
    // 3
    else if (actual_angle <= -90) {
        relative_x = -1 * cos(d2r(180 + actual_angle)) * range;
        relative_y = -1 * sin(d2r(180 + actual_angle)) * range;
    }
    // 4
    else {
        relative_x =      cos(d2r(-actual_angle)) * range;
        relative_y = -1 * sin(d2r(-actual_angle)) * range;
    }

    struct coord hit_pos;
    hit_pos.x = static_cast<int>(round((robot_x + relative_x) / k_granularity));
    hit_pos.y = static_cast<int>(round((robot_y + relative_y) / k_granularity));
    
    unordered_map<coord, cell_params>::iterator it = occupancy_grid.find(hit_pos);
    
    struct cell_params hit_cell;
    // Create a new entry
    if (it == occupancy_grid.end()) {
        hit_cell.num_hits = 1;
        hit_cell.num_misses = 0;
        hit_cell.last_updated = 0;    // NOTE: Update this for time decay
        occupancy_grid[hit_pos] = hit_cell;

    }
    // Update existing entry
    else {
        hit_cell = it->second;
        hit_cell.num_hits++;
        occupancy_grid[hit_pos] = hit_cell;        
    }

    int width, height;
    gtk_window_get_size(GTK_WINDOW(window), &width, &height);
    int center_x, center_y;
    center_x = static_cast<int>(width / 2);
    center_y = static_cast<int>(height / 2);

    // Gets every point along the "hit" line that didn't actually hit
    vector<coord> vacant_points = bresenham(robot_pos, hit_pos);
   
    for (coord& c: vacant_points) {
        
        unordered_map<coord, cell_params>::iterator vacant_it = occupancy_grid.find(c);
        struct cell_params update_cell;

        if (vacant_it == occupancy_grid.end()) {
            update_cell.num_hits = 0;
            update_cell.num_misses = 1;
            update_cell.last_updated = 0;    // NOTE: Update this for time decay
            occupancy_grid[c] = update_cell;
        }
        // Update existing entry
        else {
            update_cell = vacant_it->second;
            update_cell.num_misses++;
            occupancy_grid[c] = update_cell;        
        }

        int light = 255 - static_cast<int>((255 * update_cell.num_hits)
                                / (update_cell.num_hits + update_cell.num_misses)); 
        draw_brush(drawing_area, center_x + 3*c.x, center_y + 3*c.y, light, light, light);
    }

    // Adds the hit cell
    int light = 255 - static_cast<int>((255 * hit_cell.num_hits)
                            / (hit_cell.num_hits + hit_cell.num_misses)); 
    draw_brush(drawing_area,center_x + 3*hit_pos.x, center_y + 3*hit_pos.y, light, light, light);



//    guard _gg(mx);
//
//    // Window shenanegans
//    int ww, hh;
//    gtk_window_get_size(GTK_WINDOW(window), &ww, &hh);
//    //cout << "window: " << ww << "," << hh << endl;
//
//    int dd = min(ww, hh) / 2;
//
//    angle += (M_PI / 2.0);
//    float dx = 0.5 * range * cos(angle);
//    float dy = 0.5 * range * sin(angle);
//
//    /*
//    cout << "rr,aa; dx,dy = "
//         << range << "," << angle << "; "
//         << dx << "," << dy << endl;
//    */
//
//    int xx = dd + (dd*dx);
//    int yy = hh - (dd + (dd*dy));
//
//    /*
//    cout << "ww,hh; xx,yy = "
//         << ww << "," << hh << "; "
//         << xx << "," << yy << endl;
//    */
//
//    draw_brush(drawing_area, xx, yy);
    //GDK_THREADS_LEAVE();
}


// Gets the difference between two angles accounting for circling around
float diff(float deg1, float deg2) {
    float d = deg1 - deg2;
    d = d > 180 ? d - 360 : d;
    d = d < -180 ? d + 360 : d;
    return d;
}

// Calculates the empty points in a line between two points using bresenham's algorithm
// Implementation inspired by MyTechnoWorld
vector<coord> bresenham(coord pos1, coord pos2) {
    int x1 = pos1.x;
    int y1 = pos1.y;
    int x2 = pos2.x;
    int y2 = pos2.y;

    int dx, dy, p;
    dx = x2 - x1;
    dy = y2 - y1;
    p = 2 * (dy) - (dx);

    vector<coord> ret = {};

    while (x1 <= x2) {
        if (p < 0) {
            x1++;
            p = p + 2 * dy;
        }
        else {
            x1++;
            y1++;
            p = p + 2 * (dy - dx);
        }
        struct coord to_add;
        to_add.x = x1;
        to_add.y = y1;
        ret.push_back(to_add);
    }
    return ret;
}

//////////////////////////////////////////
// END OF OCCUPANCY GRID IMPLEMENTATION //
//////////////////////////////////////////

int
viz_run(int argc, char **argv)
{
    XInitThreads();

    GtkApplication *app;

    app = gtk_application_new("site.ntuck-neu.brain", G_APPLICATION_FLAGS_NONE);
    g_signal_connect(app, "activate", G_CALLBACK(activate), NULL);
    int status = g_application_run(G_APPLICATION(app), argc, argv);
    g_object_unref(app);
    return status;
}

