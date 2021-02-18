#trying to figure out how many links to create so that we can evenly divide the links into subsets such that a N-link arm can be turned into a (n<N)-link arm

#idea: find optimal locking so that energy is minimized for a reaching task / walking task

from math import gcd # Python versions 3.5 and above
#from fractions import gcd # Python versions below 3.5
from functools import reduce # Python version 3.x

def lcm(denominators):
    return reduce(lambda a,b: a*b // gcd(a,b), denominators)

'''
Example:
>>> lcm([100, 200, 300])
>>> lcm(list(range(1,9)))
'''
#find number of links necessary to have arm be able to be decomposed between 1-link to 7-links
lcm(list(range(1,7)))


