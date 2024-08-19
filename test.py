dic={
"r1" : 1498.15,
"r2" : -0.0675591,
"rn" : 359.931,
"rx" : 5965.2,
"ry" : -816,
"rz" : 3501.58
}

print("point 1")
print(f"or2 42 {dic['rz']} 10;or2 42 {dic['rz']} 80;")
print(f"or2 45 {dic['ry']} 10;or2 45 {dic['ry']} 80;")
print(f"or2 47 {dic['r1']} 10;or2 47 {dic['r1']} 80;")
print(f"or2 41 {dic['rx']} 10;or2 41 {dic['rx']} 80;")
print(f"or2 1 {dic['rn']} 100")
print("insert")
print(f"or2 1 {dic['rn']+36*20} 10")
print("home")
print(f"or2 1 0 100")
print(f"or2 47 0 10;or2 47 0 80;")
print(f"or2 41 0 10;or2 41 0 80;")
print(f"or2 45 0 10;or2 45 0 80;")
print(f"or2 42 0 10;or2 42 0 80;")

