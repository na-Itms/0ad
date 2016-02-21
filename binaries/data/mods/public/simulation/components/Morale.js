function Morale() {}

Morale.prototype.Schema =
	"<element name='PointsPerMember' a:help='Morale points given for each unit in the formation.'>" +
		"<ref name='nonNegativeDecimal'/>" +
	"</element>" +
	"<element name='BonusPoints' a:help='Bonus morale points, on top of the points for each member.'>" +
		"<ref name='nonNegativeDecimal'/>" +
	"</element>" +
	"<element name='RegenRate' a:help='Morale regeneration rate.'>" +
		"<ref name='nonNegativeDecimal'/>" +
	"</element>";

Morale.prototype.Init = function()
{
	this.pointsPerMember = +this.template.PointsPerMember;
	this.bonusPoints = +this.template.BonusPoints;
	this.regenRate = +this.template.RegenRate;

	this.maxMoralePoints = 0;
	this.moralePoints = 0;

	this.memberCount = 0;

	this.CheckRegenTimer();
};

Morale.prototype.GetMaxMoralePoints = function()
{
	return this.maxMoralePoints;
};

Morale.prototype.GetMoralePoints = function()
{
	return this.moralePoints;
};

Morale.prototype.GetRegenRate = function()
{
	return this.regenRate;
};

Morale.prototype.ExecuteRegeneration = function()
{
	let regen = this.GetRegenRate();

	if (regen > 0)
		this.Increase(regen);
	else
		this.Reduce(-regen);
};

/*
 * Check if the regeneration timer needs to be started or stopped
 */
Morale.prototype.CheckRegenTimer = function()
{
	// check if we need a timer
	if (this.GetRegenRate() == 0 ||
	    this.GetMoralePoints() == this.GetMaxMoralePoints() && this.GetRegenRate() >= 0 ||
	    this.GetMoralePoints() == 0)
	{
		// we don't need a timer, disable if one exists
		if (this.regenTimer)
		{
			let cmpTimer = Engine.QueryInterface(SYSTEM_ENTITY, IID_Timer);
			cmpTimer.CancelTimer(this.regenTimer);
			this.regenTimer = undefined;
		}
		return;
	}

	// we need a timer, enable if one doesn't exist
	if (this.regenTimer)
		return;
	let cmpTimer = Engine.QueryInterface(SYSTEM_ENTITY, IID_Timer);
	this.regenTimer = cmpTimer.SetInterval(this.entity, IID_Morale, "ExecuteRegeneration", 1000, 1000, null);
};

Morale.prototype.Reduce = function(amount)
{
	let oldMoralePoints = this.moralePoints;
	if (amount >= this.moralePoints)
	{
		// If we have no morale anymore, then disband.
		// The controller will exist a little while after calling
		// Disband so this might get called multiple times.
		if (!this.moralePoints)
			return;

		// TODO having a sound and/or animations for forced disbanding would be neat!

		let cmpFormation = Engine.QueryInterface(this.entity, IID_Formation);
		cmpFormation.Disband();

		this.moralePoints = 0;
	}
	else
		this.moralePoints -= amount;

	Engine.PostMessage(this.entity, MT_MoraleChanged, { "from": oldMoralePoints, "to": this.moralePoints });
};

Morale.prototype.Increase = function(amount)
{
	if (this.moralePoints == this.GetMaxMoralePoints())
		return;

	let oldMoralePoints = this.moralePoints;
	this.moralePoints = Math.min(this.moralePoints + amount, this.GetMaxMoralePoints());

	Engine.PostMessage(this.entity, MT_MoraleChanged, { "from": oldMoralePoints, "to": this.moralePoints });
};

Morale.prototype.AddMember = function(memory)
{
	// If this is the first member in the new formation, init the maximum now
	// Also make sure we start with a non-zero morale
	if (this.maxMoralePoints == 0)
	{
		this.moralePoints = 1;
		this.maxMoralePoints = this.bonusPoints;
	}

	this.maxMoralePoints += this.pointsPerMember;
	++this.memberCount;

	if (memory < 0) // No memory, use the maximum
		this.Increase(this.pointsPerMember)
	else
		this.Increase(memory);
};

Morale.prototype.RemoveMember = function()
{
	this.maxMoralePoints -= this.pointsPerMember;
	--this.memberCount;

	this.Reduce(this.pointsPerMember);
};

Morale.prototype.ComputeIndividualMorale = function()
{
	if (!this.memberCount)
		return 0;

	let amountWithoutBonus = this.moralePoints - this.bonusPoints;
	return Math.max(0, amountWithoutBonus) / this.memberCount;
};

Morale.prototype.OnMoraleChanged = function()
{
	this.CheckRegenTimer();
};

Engine.RegisterComponentType(IID_Morale, "Morale", Morale);
