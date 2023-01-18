extends KinematicBody

export var MaxSpeed := 100
export var JumpFore := 15
export var Gravity := 50

var IsJumping := false
var Velocity := Vector3.ZERO
var SnapVector := Vector3.DOWN
var Direction := Vector3.ZERO

onready var springArm = get_node("SpringArm")
onready var pivot = get_node("Pivot")
onready var model: Spatial = get_node("Pivot/Robot")
onready var skeleton = get_node("Pivot/Robot/Armature/Skeleton")
onready var bodyRay: RayCast = get_node("Pivot/Robot/Armature/SKeleton/BodyRay")
onready var leftTarget = get_node("Pivot/Robot/TargetLeft")
onready var rightTarget = get_node("Pivot/Robot/TargetRight")
onready var rayTargetLeft = get_node("Pivot/Robot/TargetLeft/RayTargetLeft")
onready var rayTargetRight = get_node("Pivot/Robot/TargetRight/RayTargetRight")
onready var moveTarget = get_node("moveTarget")

onready var targetContL = get_node("Pivot/target_container_left")
onready var targetContR = get_node("Pivot/target_container_right")

onready var raycastTargetL = get_node("Pivot/raycast_target_left")
onready var raycastTargetR = get_node("Pivot/raycast_target_right")

onready var raycastFootL = get_node("Pivot/Robot/Armature/Skeleton/left_foot/raycast_left")
onready var raycastFootR = get_node("Pivot/Robot/Armature/Skeleton/right_foot/raycast_right")

var _lerp: float = 0


func _ready() -> void:
	targetContL.set_as_toplevel(true)
	targetContR.set_as_toplevel(true)

	var ikLeft = get_node("Pivot/Robot/Armature/Skeleton/ik_left_foot")
	var ikRight = get_node("Pivot/Robot/Armature/Skeleton/ik_right_foot")

	ikLeft.start()
	ikRight.start()


func _physics_process(delta: float) -> void:
	Move(delta)

	#if(raycastFootL.is_colliding()):
	#elif(raycastFootR.is_colliding()):
	#IKStep(targetContL, leftTarget, 6.0, raycastFootL, delta)
	IKStep(targetContR, rightTarget, 4.0, raycastFootR, delta)


func _process(delta: float) -> void:
	springArm.translation.z = self.translation.z

	RayTargetUpdate(rayTargetLeft, leftTarget)
	RayTargetUpdate(rayTargetRight, rightTarget)
	#LowerBody()


func IKStep(currPos: Position3D, targetPos: Position3D, distance: float, ray: RayCast, delta: float) -> void:
	var last = currPos.global_translation
	var new = targetPos
	var stepHeight: float = 1.1
	
	if ray.is_colliding():
		if targetPos.global_translation.distance_to(ray.get_collision_point()) >= distance:
			#currPos.global_translation = currPos.global_translation.linear_interpolate(targetPos.global_translation, 0.9)
			print(targetPos.global_translation.distance_to(last))
			
			new.global_translation = targetPos.global_translation
			_lerp = 0
		#elif targetPos.global_translation.distance_to(last) >= distance:
	#	_lerp = 0;

	if _lerp < 1:
		var footPosition = currPos.global_translation.linear_interpolate(targetPos.global_translation, _lerp)
		footPosition.y += sin(_lerp * PI) * stepHeight
		
		currPos.global_translation = footPosition
		_lerp += 2 * delta
	else:
		last = new.global_translation



func RayTargetUpdate(ray: RayCast, currPos: Position3D) -> void:
	if ray.is_colliding():
		currPos.global_translation.y = ray.get_collision_point().y


func LowerBody() -> void:
	var height = 0.5
	if bodyRay.is_colliding():
		model.global_translation.y = bodyRay.get_collision_point().y + height


func Move(delta: float) -> void:
	var moveDirection := Vector3.ZERO
	moveDirection.x = Input.get_action_strength("right") - Input.get_action_strength("left")
	moveDirection.z = Input.get_action_strength("back") - Input.get_action_strength("forward")

	if moveDirection != Vector3.ZERO:
		moveDirection = moveDirection.normalized()
		Direction = moveDirection

	Velocity.x = moveDirection.x * MaxSpeed * delta
	Velocity.z = moveDirection.z * MaxSpeed * delta
	Velocity.y -= Gravity * delta

	var justLanded = is_on_floor() and SnapVector == Vector3.ZERO
	var isJumping = is_on_floor() && Input.is_action_just_pressed("jump")

	if isJumping:
		Velocity.y = JumpFore
		SnapVector = Vector3.ZERO
	elif justLanded:
		SnapVector = Vector3.DOWN

	Velocity = move_and_slide_with_snap(Velocity, SnapVector, Vector3.UP, true)
