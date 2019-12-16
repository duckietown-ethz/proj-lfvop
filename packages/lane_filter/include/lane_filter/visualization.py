import duckietown_utils as dtu
import numpy as np
import numpy.ma as ma
from reprep.graphics.filter_scale import scale


def plot_phi_d_diagram_bgr(lane_filter, belief, phi, d, dpi=120,
                           bgcolor=dtu.ColorConstants.RGB_DUCKIETOWN_YELLOW,
                           other_phi=None, other_d=None):
    """ Returns a BGR image """

    figure_args = dict(facecolor=dtu.matplotlib_01_from_rgb(bgcolor))
    a = dtu.CreateImageFromPylab(dpi=dpi, figure_args=figure_args)

    with a as pylab:
        f_d = lambda x: 100 * x
        f_phi = np.rad2deg
        # Where are the lanes?
        lane_width = lane_filter.lanewidth
        d_max = lane_filter.d_max
        d_min = lane_filter.d_min
        phi_max = lane_filter.phi_max
        phi_min = lane_filter.phi_min
        delta_d = lane_filter.delta_d
        delta_phi = lane_filter.delta_phi

        # note transpose
        belief = belief.copy()
        zeros = belief == 0

        belief[zeros] = np.nan

        belief_image = scale(belief, min_value=0)

        x = f_d(lane_filter.d_pcolor)
        y = f_phi(lane_filter.phi_pcolor)

        z = belief_image[:, :, 0]  # just R component
        z = ma.masked_array(z, zeros)

        pylab.pcolor(x, y, np.ones(z.shape), cmap='Pastel1')

        pylab.pcolor(x, y, z, cmap='gray')

        if other_phi is not None:
            for _phi, _d in zip(other_phi, other_d):

                pylab.plot(f_d(_d), f_phi(_phi), 'go', markersize=15,
                           markeredgecolor='none',
                           markeredgewidth=3,
                           markerfacecolor='blue')

        pylab.plot(f_d(d), f_phi(phi), 'go', markersize=20,
                   markeredgecolor='magenta',
                   markeredgewidth=3,
                   markerfacecolor='none')

        pylab.plot(f_d(d), f_phi(phi), 'o', markersize=2,
                   markeredgecolor='none',
                   markeredgewidth=0,
                   markerfacecolor='magenta')

        W = f_d(lane_width / 2)
        width_white = f_d(lane_filter.linewidth_white)
        width_yellow = f_d(lane_filter.linewidth_yellow)

        pylab.plot([-W, -W], [f_phi(phi_min), f_phi(phi_max)], 'k-')
        pylab.plot([-W - width_white, -W - width_white], [f_phi(phi_min), f_phi(phi_max)], 'k-')
        pylab.plot([0, 0], [f_phi(phi_min), f_phi(phi_max)], 'k-')
        pylab.plot([+W, +W], [f_phi(phi_min), f_phi(phi_max)], 'y-')
        pylab.plot([+W + width_yellow, +W + width_yellow], [f_phi(phi_min), f_phi(phi_max)], 'y-')
        s = ''
        s += "status = %s" % lane_filter.getStatus()
        s += "\nphi = %.1f deg" % f_phi(phi)
        s += "\nd = %.1f cm" % f_d(d)
        s += "\nentropy = %.4f" % lane_filter.get_entropy()
        s += "\nmax = %.4f" % belief.max()
        s += "\nmin = %.4f" % belief.min()

        if other_phi is not None:
            s += '\n Other answers:'
            for _phi, _d in zip(other_phi, other_d):
                s += "\nphi = %.1f deg" % f_phi(_phi)
                s += "\nd = %.1f cm" % f_d(_d)

        y = f_phi(phi_max) - 10
        args = dict(rotation=-90, color='white')
        annotate = True
        if annotate:
            pylab.annotate(s, xy=(0.7, 0.35), xycoords='figure fraction')
            pylab.annotate("in middle of right lane", xy=(0, y), **args)
            pylab.annotate("on right white tape", xy=(-W, y), **args)
            pylab.annotate("on left yellow tape", xy=(+W, y), **args)
            pylab.annotate("in other lane", xy=(+W * 1.3, y), **args)

        pylab.axis([f_d(d_min), f_d(d_max), f_phi(phi_min), f_phi(phi_max)])

        pylab.ylabel('phi: orientation (deg); cell = %.1f deg' % f_phi(delta_phi))
        pylab.xlabel('d: distance from center line (cm); cell = %.1f cm' % f_d(delta_d))

    return a.get_bgr()
