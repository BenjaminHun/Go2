def a():
    b()  # Error: b is not defined yet!

a()

def b():
    print("Hello from b!")