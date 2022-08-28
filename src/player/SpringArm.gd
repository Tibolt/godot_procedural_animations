extends SpringArm

export var MouseSens := 0.05

func _ready():
	set_as_toplevel(true)
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
	
func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		rotation_degrees.x += event.relative.y * MouseSens
		rotation_degrees.x = clamp(rotation_degrees.x, -180.0, -1.0)
		
		rotation_degrees.y -= event.relative.x * MouseSens
		#rotation_degrees.y = wrapf(rotation_degrees.x, 0, 360.0)



