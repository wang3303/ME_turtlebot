import numpy as np


def wrapToPi(a):
	if isinstance(a, list):
		return [(x + np.pi) % (2 * np.pi) - np.pi for x in a]
	return (a + np.pi) % (2 * np.pi) - np.pi


def wrapTo2Pi(a):
	# wrap to 0-2pi
	if isinstance(a, list):
		return [(x + 2 * np.pi) % (2 * np.pi) for x in a]
	return (a + 2 * np.pi) % (2 * np.pi)
