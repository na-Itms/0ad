function FormationAttack() {}

FormationAttack.prototype.Schema =
	"<empty/>";

FormationAttack.prototype.Init = function()
{
};

FormationAttack.prototype.GetAttackTypes = function()
{
	return [];
};

FormationAttack.prototype.GetRange = function(target)
{
	var result = {"min": 0, "max": -1};
	var cmpFormation = Engine.QueryInterface(this.entity, IID_Formation);
	if (!cmpFormation)
	{
		warn("FormationAttack component used on a non-formation entity");
		return result;
	}
	var members = cmpFormation.GetMembers();
	for each (var ent in members)
	{
		var cmpAttack = Engine.QueryInterface(ent, IID_Attack);
		if (!cmpAttack)
			continue;

		var type = cmpAttack.GetBestAttackAgainst(target);
		if (!type)
			continue;

		// Take the minimum max range and the minimum min range (to not get impossible situations)
		var range = cmpAttack.GetRange(type);

		if (range.max > result.max || range.max < 0)
			result.max = range.max;
		if (range.min < result.min)
			result.min = range.min;
	}
	// add half the formation size, so it counts as the range for the units on the first row
	var extraRange = cmpFormation.GetSize().depth/2;

	if (result.max >= 0)
		result.max += extraRange;
	result.min += extraRange;
	return result;
};

Engine.RegisterComponentType(IID_Attack, "FormationAttack", FormationAttack);
