import collections
import os

from numpy.testing.utils import assert_equal, assert_almost_equal

import duckietown_utils as dtu
from grid_helper.grid_helper_visualization import grid_helper_plot
from grid_helper.voting_grid import GridHelper, array_as_string_sign,\
    array_as_string
import numpy as np


@dtu.unit_test
def compare_faster():
    variables = collections.OrderedDict()
    variables['alpha'] = dict(min=-180, max=180, description="angle",
                          resolution=5,
                          units='deg', units_display='deg')
    variables['r'] = dict(min=3, max=5, description="distance", resolution=0.1,
                          units='m', units_display='cm')
    # this will fail if precision is float32
    gh = GridHelper(variables, precision='float64')
    val_fast = gh.create_new()
    val_fast.fill(0)
    val_slow = gh.create_new()
    val_slow.fill(0)
    
    od = dtu.get_output_dir_for_test()
    
    F = 1
    
    alpha0 = 7
    # r0 = 4
    r0 = 4.1
    w0 = 1.
    value = dict(alpha=alpha0, r=r0)
    gh.add_vote(val_slow, value, w0, F)
    
    assert_equal(np.sum(val_slow > 0), 9)
    
    values = np.zeros((2, 1))
    values[0, 0] = alpha0
    values[1, 0] = r0
    weights = np.zeros(1)
    weights[0]= w0
    gh.add_vote_faster(val_fast, values, weights, F)
    
    assert_equal(np.sum(val_fast > 0), 9)
    
    d = grid_helper_plot(gh, val_slow)
    fn = os.path.join(od, 'compare_faster_slow.jpg')
    dtu.write_data_to_file(d.get_png(), fn)
    
    d = grid_helper_plot(gh, val_fast)
    fn = os.path.join(od, 'compare_faster_fast.jpg')
    dtu.write_data_to_file(d.get_png(), fn)
    
    D = val_fast - val_slow
    diff = np.max(np.abs(D))
    print('diff: %r' % diff)
    if diff > 1e-8:
        print dtu.indent(array_as_string_sign(val_fast), 'val_fast ')
        print dtu.indent(array_as_string_sign(val_slow), 'val_slow ')
        print dtu.indent(array_as_string_sign(D), 'Diff ')
        print('non zero val_fast: %s' %  val_fast[val_fast > 0])
        print('non zero val_slow: %s' %  val_slow[val_slow > 0])
        
    assert_almost_equal(val_fast, val_slow)




@dtu.unit_test
def compare_faster2():
    variables = collections.OrderedDict()
    variables['alpha'] = dict(min=-180, max=180, description="angle",
                          resolution=5,
                          units='deg', units_display='deg')
    variables['r'] = dict(min=3, max=5, description="distance", resolution=0.2,
                          units='m', units_display='cm')
    # this will fail if precision is float32
    gh = GridHelper(variables, precision='float64')
    val_fast = gh.create_new()
    val_fast.fill(0)
    val_slow = gh.create_new()
    val_slow.fill(0)
    
    od = dtu.get_output_dir_for_test()
    
    F = 1
    alpha0 = 177
    # r0 = 4
    r0 = 4.1
    w0 = 1
    value = dict(alpha=alpha0, r=r0)
    gh.add_vote(val_slow, value, w0, F)
    
    values = np.zeros((2, 1))
    values[0, 0] = alpha0
    values[1, 0] = r0
    weights = np.zeros(1)
    weights[0]= w0
    gh.add_vote_faster(val_fast, values, weights, F)
    
    print('sum slow: %s' % np.sum(val_slow))
    print('sum fast: %s' % np.sum(val_fast))
    d = grid_helper_plot(gh, val_slow)
    fn = os.path.join(od, 'compare_faster_slow.jpg')
    dtu.write_data_to_file(d.get_png(), fn)
    
    d = grid_helper_plot(gh, val_fast)
    fn = os.path.join(od, 'compare_faster_fast.jpg')
    dtu.write_data_to_file(d.get_png(), fn)
    
    f = lambda x: ' %5f' % x if x else '    '
    print dtu.indent(array_as_string(val_fast, f), 'val_fast ')
    print dtu.indent(array_as_string(val_slow, f), 'val_slow ')
    
    assert_almost_equal(val_fast, val_slow)


if __name__ == '__main__':
    dtu.run_tests_for_this_module()
