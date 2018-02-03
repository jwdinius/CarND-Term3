import matplotlib.pyplot as plt

pedal_cmd = []
for line in open("output.txt"):
	a = line.split()
	if (len(a) > 1):
		if a[0] == "pedal_cmd:":
			pedal_cmd.append(float(a[1]))
			#print float(a[1])

plt.figure()
plt.plot(pedal_cmd,label='pedal_cmd')
plt.title('pedal_cmd (filtered)')
plt.show()