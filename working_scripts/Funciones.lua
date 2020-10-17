---------------
-- FUNCTIONS --
---------------

--Returns a random value from a normal distribution of mean
--med and deviation deviation with the Box-Muller method
normrnd = function(med, desv)
    local U = math.random()
    local V = math.random()
    local X = math.sqrt(-2*math.log(U))*math.cos(2*math.pi*V)
    return (X*desv)+med
end

--FUNCTIONS FOR ACTUATORS

--Provides speed to the Motor
On=function(inInts,inFloats,inStrings,inBuffer)
	if(finish ~= 1) then -- parametric finish
		local speed
		if(inInts[2] > MAX_SPEED) then
			speed = MAX_SPEED
		elseif(inInts[2] < -MAX_SPEED) then
			speed = -MAX_SPEED
		else
			speed = inInts[2]
		end

		if(inInts[1] == 1) then -- If it is the value of the motor constant B
            -- We activate engine B
			sim.setJointTargetVelocity(motorB, speed)
			sim.setJointForce(motorB, MOTION_TORQUE)
		elseif(inInts[1] == 2) then -- f it is the value of the motor constant C
			--We activate engine C
			sim.setJointTargetVelocity(motorC, speed)
			sim.setJointForce(motorC, MOTION_TORQUE)
		elseif(inInts[1] == 3) then -- If is the value of the constant of both motors
			--We activate both engines
			sim.setJointTargetVelocity(motorB, speed)
			sim.setJointTargetVelocity(motorC, speed)
			sim.setJointForce(motorB, MOTION_TORQUE)
			sim.setJointForce(motorC, MOTION_TORQUE)
		end
	end
	return {},{},{},''
end

--Stops engines
Off=function(inInts,inFloats,inStrings,inBuffer)
	if(finish ~= 1) then -- parametric finish
		if(inInts[1] == 1) then -- If it is the value of the motor constant B
            -- We activate engine B
			sim.setJointTargetVelocity(motorB, 0)
			sim.setJointForce(motorB, REST_TORQUE)
		elseif(inInts[1] == 2) then -- f it is the value of the motor constant C
			--We activate engine C
			sim.setJointTargetVelocity(motorC, 0)
			sim.setJointForce(motorC, REST_TORQUE)
		elseif(inInts[1] == 3) then -- If is the value of the constant of both motors
			--We activate both engines
			sim.setJointTargetVelocity(motorB, 0)
			sim.setJointTargetVelocity(motorC, 0)
			sim.setJointForce(motorB, REST_TORQUE)
			sim.setJointForce(motorC, REST_TORQUE)
		end
	end
	return {},{},{},''
end

-- FUNCIONES PARA SENSORES

--Returns the rotation of Motor B
MotorRotationCountB=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- ParÃ¡metro finish
        local rotation = sim.getJointPosition(motorB) -- recogemos el angulo de rotacion
        if(rotation < 0) then -- y agustamos segÃºn el origen mediante la variable reset
            rotation = math.ceil(math.deg(rotation-resetB))
        else
            rotation = math.floor(math.deg(rotation-resetB))
        end
        return {rotation},{},{},'' -- devolvemos el resultado.
    else
        return{},{},{},''
    end
end

--Returns the rotation of Motor C
MotorRotationCountC=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- ParÃ¡metro finish
        local rotation = sim.getJointPosition(motorC) -- recogemos el angulo de rotacion
        if(rotation < 0) then -- y agustamos segÃºn el origen mediante la variable reset
            rotation = math.ceil(math.deg(rotation-resetC))
        else
            rotation = math.floor(math.deg(rotation-resetC))
        end
        return {rotation},{},{},'' -- devolvemos el resultado.
    else
        return{},{},{},''
    end
end

--Reset the encoders from the current position
ResetRotationCount=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- ParÃ¡metro finish
        local motor = inInts[1] -- se recoje el tipo de motor
        if(motor == 1)then -- si es B establece el origen de B
            resetB = sim.getJointPosition(motorB)
        elseif(motor == 2)then -- si no, establece el origen de C
            resetC = sim.getJointPosition(motorC)
        end
    end
    return {},{},{},''
end



--Returns the information of the touch sensor
SensorTouch = function (inInts, inFloats, inStrings, inBuffer)
     if (finish ~ = 1) then -- finish parameter
     local res - result variable
         local state, forceVector = sim.readForceSensor (bumper) -- sensor data is read
         -- and the state and the
         -- force vector
     if (state> 0) then - if it has detected a click
             if ((forceVector [3]> 1)) then - we check if a force of more than
 -- 1N and we will return as result 1
                 res = 1;
             else - otherwise we will return 0
                 res = 0;
             end
         else - otherwise we will return 0
             res = 0;
         end
return {res}, {}, {}, '' - the result is returned
     else
         return {}, {}, {}, ''
     end
end

--Devuelve la informacion del sensor tactil
SensorTouch=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- parÃ¡metro finish
		local res -- variable de resultado
        local state, forceVector = sim.readForceSensor(bumper) -- se leen los datos del sensor
		                                                      -- y se obtienen el estado y el
															  -- vector de fuerza
		if (state > 0) then -- si ha detectado pulsaciÃ³n
            if ((forceVector[3]>1))then -- comprobamos si se ha ejercido una fuerza de mas de
			                            -- 1N y devoleremos como resultado 1
                res = 1;
            else -- en otro caso devolveremos 0
                res = 0;
            end
        else -- en otro caso devolveremos 0
            res = 0;
        end
		return {res},{},{},'' -- se devuelve el resultado
    else
        return {},{},{},''
    end
end

--Returns the sonar information
SensorSonar=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- parÃ¡metro finish
        local detect, dist = sim.readProximitySensor(sonar) -- the parameters are read
														   -- proximity sensor
		local aux = 255 -- the value of "not detected" is assigned by default
		if(dist ~= nil and detect > 0) then -- if you have successfully detected
			aux = math.floor(dist*1000+normrnd(0,0.5))/10 -- adjustments are made
		end
        return {},{aux},{},'' -- the data is returned
    else
        return {},{},{},''
    end
end

-- Returns the mean value of light intensity
SensorLight=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- parÃ¡metro finish
        local auxData = select(2,sim.readVisionSensor(sensorColorLR))
		local desv = 0.2905 + 0.3197*auxData[11];
		local aux = 7+64*auxData[11]+normrnd(0,desv) -- Se escala el valor y se le aplica el error
        return {math.floor(aux)},{},{},''
    else
        return {},{},{},''
    end
end

-- Returns the average rgb values and depth) of the color sensor
SensorColor=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- parÃ¡metro finish
        local auxData = select(2,sim.readVisionSensor(sensorColorLR))
        return {},{auxData[12], auxData[13], auxData[14], auxData[15]},{},''
    else
        return {},{},{},''
    end
end

-- returns the angular velocity
SensorGyroVA=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- parÃ¡metro finish
        local gyroZ = sim.getIntegerSignal('gyroZ_velocity')
		-- the angular velocity of the gyroscope is collected and returned
        return {gyroZ},{},{},''
    else
        return {},{},{},''
    end
end

-- returns the rotated angle
SensorGyroA=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- parÃ¡metro finish
        local gyroZ = sim.getIntegerSignal('gyroZ_angle')
		-- the rotated angle of the gyroscope is collected and returned
        return {gyroZ},{},{},''
    else
        return {},{},{},''
    end
end

-- resetea el angulo girado
ResetGyroA=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- parÃ¡metro finish
        sim.setIntegerSignal('gyroZ_angle_reset', 1)
		-- se resetea el conteo de Ã¡ngulos
    end
        return {},{},{},''
end

-- INTERFACE FUNCTIONS

--AÃ±ade un texto en pantalla dependiendo de la linea indicada
TextOut=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- ParÃ¡metro finish
		if(inInts[1] >= 1 and inInts[1] <= 8) then
			simSetUIButtonLabel(UI, inInts[1], inStrings[1])
		end
    end
    return {},{},{},''
end

--limpia la pantalla (consola auxiliar)
ClearScreen=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- ParÃ¡metro finish
        for i=1,8 do
            simSetUIButtonLabel(UI, i, '')
        end
    end
    return {},{},{},''
end

StatusLight=function(inInts,inFloats,inStrings,inBuffer)
	if(finish ~= 1) then -- ParÃ¡metro finish
		stroke = inInts[2]
		if (inInts[1] >= 1 and inInts[1] <= 4) then
			color = LIGHTS[inInts[1]]
		else
			color = LIGHTS[4]
		end
		simSetUIButtonColor(UI, 17, color, nil)
		simSetUIButtonColor(UI, 18, color, nil)
	end
	return {},{},{},''
end

ButtonPressed=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- parÃ¡metro finish
		return {button},{},{},''
	else
		return {},{},{},''
	end
end

-- OTRAS FUNCIONES

--Devuelve el valor del paso de simulaciÃ³n en segundos
SimulationTimeStep=function(inInts,inFloats,inStrings,inBuffer)
	return {},{sim.getSimulationTimeStep()},{},''
end

--FunciÃ³n vacia que nos servirÃ¡ para realizar esperas
EmptyFunction=function(inInts,inFloats,inStrings,inBuffer)
	return {},{},{},''
end

--Devuelve el tiempo actual de ejecucion (tiempo de simulaciÃ³n en ms)
CurrentTick=function(inInts,inFloats,inStrings,inBuffer)
    if(finish ~= 1) then -- ParÃ¡metro finish
        return {math.floor(sim.getSimulationTime())},{},{},''
    else
        return {},{},{},''
    end
end

--Finaliza el programa
Stop = function(inInts,inFloats,inStrings,inBuffer)
	-- volvemos a cero el resto de variables
    waitTimeTotal = 0
    resetB = 0
    resetC = 0
    lastTime = 0
	button = 0
	-- limpiamos la consola auxiliar y las luces de estado
	ClearScreen({},{},{},'')
	StatusLight({4, 0},{},{},'')
	-- paramos los motores
    sim.setJointTargetVelocity(motorB, 0)
    sim.setJointTargetVelocity(motorC, 0)
	-- limpiamos todas las seÃ±ales
    sim.clearFloatSignal(nil)
    sim.clearIntegerSignal(nil)
    sim.clearStringSignal(nil)
	-- si obtenemos un valor negativo en el argumento,
	-- quiere decir que el programa de matlab no se ha finalizado correctamente
	if(inInts[1] < 0) then
		sim.addStatusbarMessage('HA OCURRIDO UN ERROR EN EL CLIENTE A LA HORA DE FINALIZAR EL PROGRAMA')
	end
	finish = 1 -- variable global finish para impedir ejecuciones de funciones y fases del robot
    return {},{},{},''
end

----------------------------
-- FASES DE LA SIMULACIÃN --
----------------------------

-- PARTE DE INICIALIZACIÃN
if (sim_call_type==sim.syscb_init) then
	-- Constantes
	MOTION_TORQUE = 0.2 -- Torque en movimiento de los motores
	REST_TORQUE = 0.4  --Torque en parada de los motores
	MAX_SPEED = 18.326 -- MÃ¡xima velocidad para los motores
	LIGHTS = {{0, 1, 0}, {0.89, 0.54, 0.23}, {1, 0, 0}, {0.85, 0.85, 0.85}} -- Array de colores para la luz de estado
	PUERTO_API_REMOTA = sim.getScriptSimulationParameter(sim.handle_self, 'PUERTO_API_REMOTA', false) -- Puerto de api remota

	-- Variables globales
    lastTime = 0 -- Tiempo de simulaciÃ³n anterior
	currentTime = 0 -- Tiempo de simulaciÃ³n actual
    finish = 0 -- variable que acaba con la ejecuciÃ³n de las fases y no permite el uso de funciones; usado en la funciÃ³n close.
    resetB = 0 -- variable que guarda el Ãºltimo valor del encoder A llamado por la funcion ResetRotationCount
    resetC = 0 -- variable que guarda el Ãºltimo valor del encoder C llamado por la funcion ResetRotationCount
	button = 0 -- variable que guarda el Ãºltimo ID del boton pulsado
	stroke = 0 -- variable que indica si la luz de estado debe parpadear (0 no, 1 sÃ­)
	turn = 0 -- variable que indica que la luz de estado debe apagarse o encenderse si esta parpadea
	timestroke = 0 -- tiempo en un estado de parpadeo para la luz de estado
	color = LIGHTS[4] -- color actual de la luz de estado (apagado)

	-- Inicio de seÃ±ales
    sim.setIntegerSignal('SendStop',0) -- esta seÃ±al se encarga de indicarle a Matlab que el wait ha finalizado

	-- Referencias de objetos
    motorB = sim.getObjectHandle('Motor_B') -- Referencia del motor_B para EV3
    motorC = sim.getObjectHandle('Motor_C') -- Referencia para el motor
    bumper = sim.getObjectHandle('Bumper') -- Referencia del sensor tÃ¡ctil
    sonar = sim.getObjectHandle('Sonar') -- Referencia del sensor ultrasÃ³nico
    sensorColorLR = sim.getObjectHandle('Sensor_Color_LR') -- Referencia para el modo luz reflejada del sensor de color
    sensorColorRC = sim.getObjectHandle('Sensor_Color_RC') -- Referencia para el modo color del sensor de color

	-- Referencia de interfaz
	UI = simGetUIHandle('Interfaz_EV3')

	-- Inicio del servidor de API remota para el robot
	simRemoteApi.start(PUERTO_API_REMOTA, 1300, false, false)
end

-- PARTE DE ACTUACIÃN
if (sim_call_type==sim.syscb_actuation and finish ~= 1) then

end


-- PARTE DE SONDEO
if (sim_call_type==sim.syscb_sensing and finish ~= 1) then
    currentTime = sim.getSimulationTime()

    -- Listener de los botones del ladrillo  (escribe en la variable el ultimo boton pulsado)
	local event, state = simGetUIEventButton(UI)
	if(event ~= -1) then -- si se ha producido un evento de pulsaciÃ³n o de soltar un botÃ³n de la interfaz
		if(state[2] == 1) then -- si se ha pulsado, escribimos el id del boton pulsado
			button = event - 10
		else -- si se ha dejado de pulsar, escribimos 0 que indica que ningÃºn botÃ³n se esta pulsando
			button = 0
		end
	end

	-- Controlador del parpadeo del led de estado
	if(stroke == 1) then --si se ha indicado parpadeo
		timestroke = timestroke + (currentTime-lastTime) -- calculamos el tiempo que el led lleva en un estado
		if(timestroke >= 0.5) then -- si se ha superado el tiempo establecido para el estado
			if(turn == 0) then -- si estÃ¡n apagadas, las encendemos con el color indicado
				simSetUIButtonColor(UI, 17, color, nil)
				simSetUIButtonColor(UI, 18, color, nil)
				turn = 1 -- cambiamos de estado
			else -- en otro caso, las apagamos
				simSetUIButtonColor(UI, 17, LIGHTS[4], nil)
				simSetUIButtonColor(UI, 18, LIGHTS[4], nil)
				turn = 0 --  cambiamos de estado
			end
			timestroke = 0
		end
	end

	lastTime=currentTime -- almacenamos el tiempo actual para utilizarlo en el siguiente paso de simulaciÃ³n
end

-- PARTE DE RESTAURACIÃN
if (sim_call_type==sim.syscb_cleanup) then
	-- se apagan las luces de estado
	simSetUIButtonColor(UI, 17, LIGHTS[4], nil)
	simSetUIButtonColor(UI, 18, LIGHTS[4], nil)
	-- se detiene el servidor de API remota de este robot
	simRemoteApi.stop(PUERTO_API_REMOTA)
end
