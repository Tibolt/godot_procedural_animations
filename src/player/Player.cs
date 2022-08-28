using Godot;
using System;

public class Player : KinematicBody
{
	[Export]
	public int MaxSpeed = 100;
	[Export]
	public int JumpForce = 15;
	[Export]
	public int Gravity = 50;
	public bool IsJumping = false;
	public Vector3 Velocity = Vector3.Zero;
	public Vector3 SnapVector = Vector3.Down;
	public Vector3 Direction = Vector3.Zero;

	private SpringArm springArm {get; set;}
	private Spatial pivot {get; set;}
	private Spatial model {get; set;}
	private Position3D body {get; set;}
	private Position3D moveTarget {get; set;}

	private BoneAttachment leftFoot {get; set;}
	private BoneAttachment rightFoot {get; set;}
	private SkeletonIK ikLeftFoot {get; set;}
	private SkeletonIK ikRightFoot {get; set;}
	private Position3D targetContL {get; set;}
	private Position3D targetContR {get; set;}

    private CollisionShape targetSphereL {get;set;}

	private RayCast raycastL {get; set;}
	private RayCast raycastR {get; set;}

	private RayCast raycastTargetL {get; set;}
	private RayCast raycastTargetR {get; set;}

	public RayCast ray = new RayCast();
	
	/* private float Length = 5.0f; */


	public override void _Ready()
	{
		springArm = GetNode<SpringArm>("SpringArm");
		pivot = GetNode<Spatial>("Pivot");
		model = GetNode<Spatial>("Pivot/Robot/Body");
		body = GetNode<Position3D>("Pivot/Robot/Body");
		moveTarget = GetNode<Position3D>("moveTarget");

		leftFoot = GetNode<BoneAttachment>("Pivot/Robot/Armature/Skeleton/left_foot");
		rightFoot = GetNode<BoneAttachment>("Pivot/Robot/Armature/Skeleton/right_foot");
		ikLeftFoot = GetNode<SkeletonIK>("Pivot/Robot/Armature/Skeleton/ik_left_foot");
		ikRightFoot = GetNode<SkeletonIK>("Pivot/Robot/Armature/Skeleton/ik_right_foot");
		targetContL = GetNode<Position3D>("Pivot/target_container_left");
		targetContR = GetNode<Position3D>("Pivot/target_container_right");

        targetSphereL = GetNode<CollisionShape>("TargetCubeL");

		targetContL.SetAsToplevel(true);
		targetContR.SetAsToplevel(true);

		raycastL = GetNode<RayCast>("Pivot/Robot/Armature/Skeleton/left_foot/raycast_left");
		raycastR = GetNode<RayCast>("Pivot/Robot/Armature/Skeleton/right_foot/raycast_right");
		raycastTargetL = GetNode<RayCast>("Pivot/raycast_target_left");
		raycastTargetR = GetNode<RayCast>("Pivot/raycast_target_right");

		ikLeftFoot.Start(false);
		ikRightFoot.Start(false);
		
		body.AddChild(ray);
		ray.Enabled = true;
		ray.CastTo = Vector3.Down * 3;
	}

	public override void _PhysicsProcess(float delta)
	{
		Move(delta);
		/* Anim(); */

	}
	public void Anim()
	{
		var length = 4;
		ray.Translation = body.Translation;
		//LEFT
		var targetL = targetContL.Translation;

		/* var playerPos = new Vector3(this.GlobalTranslation.x,this.GlobalTranslation.y,this.GlobalTranslation.z); */
		var playerPos = this.GlobalTranslation;
		if(raycastL.IsColliding())
		{
			var contactPoint = raycastL.GetCollisionPoint();
			targetContL.GlobalTranslation = contactPoint;
			
			/* if(CheckFootDistance(leftFoot, this.GlobalTranslation.z - length)) */
			/* 	MoveLegToTarget(targetContL, raycastTargetL); */

		}
		else
		{
			/* if(CheckFootDistance(leftFoot, this.GlobalTranslation - Vector3.Back*length)) */
			if(CheckFootDistance(leftFoot, playerPos, length))
				MoveLegToTarget(targetContL, raycastTargetL);
		}

		// RIGHT
		if(raycastR.IsColliding())
		{
			var contactPoint = raycastR.GetCollisionPoint();
			targetContR.GlobalTranslation = contactPoint;


			/* if(CheckFootDistance(rightFoot, this.GlobalTranslation.z - length)) */
			/* 	MoveLegToTarget(targetContR, raycastTargetR); */
		}
		else
		{

			if(CheckFootDistance(leftFoot, playerPos, length))
				MoveLegToTarget(targetContR, raycastTargetR);
		}
	}
	public void UpdateIK(Position3D leg, Position3D target)
	{
		var minDistance = 5f;

		var offset = target.GlobalTranslation - leg.GlobalTranslation;
		var distanceToTarg = offset.Length();
		if(distanceToTarg < minDistance)
		{
			offset = (offset / distanceToTarg) * minDistance;
			distanceToTarg = minDistance;
		}
	}
	/* public Position3D MakeStep(Position3D legPos,Position3D tPos) */
	/* { */
	/*     if(legPos.Translation == tPos.Translation) */
	/*         return legPos; */
	/*     tPos = legPos; */
	/* } */

	public void MoveLegToTarget(Position3D leg,RayCast rayTarget)
	{
		var nextPosY = rayTarget.GetCollisionPoint().y;
		var nextPos = rayTarget.GlobalTranslation;

		var origin = leg.GlobalTransform;
		origin.origin = nextPos;
		origin.origin = new Vector3(nextPos.x +3, nextPosY, nextPos.z-3);
		/* leg.GlobalTransform = origin; */
		leg.GlobalTransform = leg.GlobalTransform.InterpolateWith(origin, 0.1f);
	}
	public bool CheckFootDistance(BoneAttachment foot, Vector3 distance, float length)
	{
		if(foot.GlobalTranslation.DistanceSquaredTo(distance) > length)
			return true;
		return false;
	}
	public override void _Process(float delta)
	{
		/* springArm.Translation = this.Translation; */
		Anim();
	}
		
	public override void _Input(InputEvent @event)
	{

		if(@event.IsActionPressed("forward"))
			Direction += new Vector3(0,0,-1);
		else if(@event.IsActionPressed("back"))
			Direction += new Vector3(0,0,1);
		if(@event.IsActionPressed("right"))
			Direction += new Vector3(1,0,0);
		else if(@event.IsActionPressed("left"))
			Direction += new Vector3(-1,0,0);
		
		if(@event.IsActionPressed("jump"))
			IsJumping = true;
	}

	public void Move(float delta)
	{
		var moveDirection = Vector3.Zero;
		moveDirection.x = Input.GetActionStrength("right") - Input.GetActionStrength("left");
		moveDirection.z = Input.GetActionStrength("back") - Input.GetActionStrength("forward");
		/* moveDirection = moveDirection.Rotated(Vector3.Up, springArm.Rotation.y).Normalized(); */

		if(moveDirection != Vector3.Zero)
		{
			moveDirection = moveDirection.Normalized();
			/* pivot.LookAt(this.Translation + moveDirection, Vector3.Up); */

			// for animation
			Direction = moveDirection;
		}

		Velocity.x = moveDirection.x * MaxSpeed * delta;
		Velocity.z = moveDirection.z * MaxSpeed * delta;
		Velocity.y -= Gravity * delta;

		var justLanded = IsOnFloor() && SnapVector == Vector3.Zero;
		var isJumping = IsOnFloor() && Input.IsActionJustPressed("jump");
		if(isJumping)
		{
			Velocity.y = JumpForce;
			SnapVector = Vector3.Zero;
		}
		else if(justLanded)
			SnapVector = Vector3.Down;

		Velocity = MoveAndSlideWithSnap(Velocity, SnapVector, Vector3.Up, true);

		/* if(Velocity.Length() > 0.2) */
		/* { */
		/* 	var lookDirection = new Vector2(Velocity.x, Velocity.z); */
		/* 	model.Rotation = new Vector3(0, lookDirection.Angle(),0); */
		/* } */


		/* Velocity = moveDirection * MaxSpeed; */
		/* Velocity = MoveAndSlide(Velocity); */
	}

}
