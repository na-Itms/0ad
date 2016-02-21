function FormationAttack() {}

FormationAttack.prototype.Schema =
	"<element name='AttackMultiplierAgainstIndividuals' a:help='The attack multiplier units in the formation get against isolated units.'>" +
		"<ref name='nonNegativeDecimal'/>" +
	"</element>";

FormationAttack.prototype.Init = function()
{
	this.attackMultiplierAgainstIndividuals = +this.template.AttackMultiplierAgainstIndividuals;
};

FormationAttack.prototype.GetAttackMultiplierAgainstIndividuals = function()
{
	return this.attackMultiplierAgainstIndividuals;
};

FormationAttack.prototype.CanAttack = function(target)
{
	let cmpThisPosition = Engine.QueryInterface(this.entity, IID_Position);
	let cmpTargetPosition = Engine.QueryInterface(target, IID_Position);
	if (!cmpThisPosition || !cmpTargetPosition || !cmpThisPosition.IsInWorld() || !cmpTargetPosition.IsInWorld())
		return false;

	// Check if the relative height difference is larger than the attack range
	// If the relative height is bigger, it means they will never be able to
	// reach each other, no matter how close they come.
	let heightDiff = Math.abs(cmpThisPosition.GetHeightOffset() - cmpTargetPosition.GetHeightOffset());
	if (heightDiff > this.GetRange().max)
		return false;

	// Formations can attack other formations or individual units
	let cmpUnitAI = Engine.QueryInterface(target, IID_UnitAI);
	if (cmpUnitAI)
		return true;

	return false;
};

FormationAttack.prototype.GetRange = function()
{
	let result = {"min": 0, "max": -1};
	let cmpFormation = Engine.QueryInterface(this.entity, IID_Formation);
	if (!cmpFormation)
	{
		err("FormationAttack component used on a non-formation entity");
		return result;
	}

	// The formation can perform a ranged attack if all members can.
	let canPerformRangedAttack = true;
	let meleeResult = {"min": 0, "max": -1};
	let rangedResult = {"min": 0, "max": -1};

	let members = cmpFormation.GetMembers();
	for (let ent of members)
	{
		let cmpAttack = Engine.QueryInterface(ent, IID_Attack);
		if (!cmpAttack)
			continue;

		// Always take the minimum max range and the minimum min range
		// (to not get impossible situations)

		if (cmpAttack.GetAttackTypes().indexOf("Melee") != -1)
		{
			let range = cmpAttack.GetRange("Melee");

			if (range.max > meleeResult.max || range.max < 0)
				meleeResult.max = range.max;
			if (range.min < meleeResult.min)
				meleeResult.min = range.min;
		}

		if (cmpAttack.GetAttackTypes().indexOf("Ranged") != -1)
		{
			let range = cmpAttack.GetRange("Ranged");

			if (range.max > rangedResult.max || range.max < 0)
				rangedResult.max = range.max;
			if (range.min < rangedResult.min)
				rangedResult.min = range.min;
		}
		else
			canPerformRangedAttack = false;
	}

	if (canPerformRangedAttack)
		result = rangedResult;
	else
		result = meleeResult;

	// add half the formation size, so it counts as the range for the units on the first row
	let extraRange = cmpFormation.GetSize().depth / 2;

	if (result.max >= 0)
		result.max += extraRange;
	result.min += extraRange;
	return result;
};

Engine.RegisterComponentType(IID_FormationAttack, "FormationAttack", FormationAttack);
