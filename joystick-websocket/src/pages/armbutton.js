import React, { useState } from 'react';

const ArmDisarmButton = ({ ws }) => {
    const [armed, setArmed] = useState(false);

    const toggleArm = () => {
        const armRequest = {
            op: 'call_service',
            service: 'arm_disarm',
            args: { data: !armed },
        };
        ws.send(JSON.stringify(armRequest));
        setArmed(!armed);
    };

    return (
        <button onClick={toggleArm}>
            {armed ? 'Disarm' : 'Arm'}
        </button>
    );
};

export default ArmDisarmButton;
