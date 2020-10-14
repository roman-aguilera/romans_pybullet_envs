def kbits(n, k):
  result = []
  for bits in itertools.combinations(range(n), k):
    s = ['0'] * n
    for bit in bits:
      s[bit] = '1'
    result.append(''.join(s))
  return result

def kbits2(n, k):
  result = []
  for bits in itertools.combinations(range(n), k):
    s = [0] * n
    for bit in bits:
      s[bit] = 1
    result.append([].join(s))
  return result


#https://code.activestate.com/recipes/425303-generating-all-strings-of-some-length-of-a-given-a/
def allstrings(alphabet, length):
	"""Find the list of all strings of 'alphabet' of length 'length'"""	
	if length == 0: return []
	c = [[a] for a in alphabet[:]]
	if length == 1: return c
	c = [[x,y] for x in alphabet for y in alphabet]
	if length == 2: return c
	for l in range(2, length):
		c = [[x]+y for x in alphabet for y in c]
	return c

def allstrings2(alphabet, length):
    """Find the list of all strings of 'alphabet' of length 'length'"""
    c = []
    for i in range(length):
        c = [[x]+y for x in alphabet for y in c or [[]]]
    return c
	
if __name__ == "__main__":
	for p in allstrings([0,1,2],4):
		print p


number_of_joints_urdf=8
number_of_free_joints=3
masks=kbits(number_of_joints_urdf, number_of_free_joints) #list of masks
print(masks)






