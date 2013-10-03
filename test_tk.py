import Tkinter as tk

class MyApp:
	"""class off an app"""
	def __init__(self, my_parent):
		self.last_invoked = None

		self.my_parent = my_parent
		self.mycontainer1 = tk.Frame(my_parent)
		self.mycontainer1.pack()

		self.button1 = tk.Button(self.mycontainer1, command=self.button1_click)
		self.button1.configure(text="OK", background="green")
		self.button1.pack(side=tk.LEFT)
		self.button1.focus_force()
		self.button1.bind("<Return>", self.button1_click_a)
		
		self.button2 = tk.Button(self.mycontainer1, command=self.button2_click)
		self.button2.configure(text="Cancel", background="red")
		self.button2.pack(side=tk.LEFT)
		self.button2.bind("<Return>", self.button2_click_a)

		self.button3 = tk.Button(self.mycontainer1, command=self.button3_click)
		self.button3.configure(text="Other", background="red")
		self.button3.pack(side=tk.LEFT)
		self.button3.bind("<Return>", self.button3_click)




	def button1_click(self):
		print 'ook clicked'
		print 'last clicked: ', self.last_invoked
		self.last_invoked = 'ok'

	def button2_click(self):
		print 'cancel'
		print 'last clicked: ', self.last_invoked
		self.last_invoked = 'cancel'

	def button3_click(self):
		print 'other clicked'
		print 'last clicked: ', self.last_invoked
		self.last_invoked = 'other'
	def button1_click_a(self, event):
		self.button1_click()

	def button2_click_a(self, event):
		self.button2_click()

def report_event(event):
	'''prints the description of an event'''
	event_name = {"2": "KeyPress", "4": "ButtonPress"}
	print 'Time: ', str(event.time)
	print 'EventType=' + str(event.type), \
		event_name[str(event.type)],\
		"EventWidgetID=" + str(event.widget), \
		"EventKeySymbol=" + str(event.keysym)

root = tk.Tk()

myapp = MyApp(root)

root.mainloop()

