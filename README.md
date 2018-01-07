# MPC-based-Energy-Management-System

The classical grid system is unidirectional and operates from the utility to the
consumer. Contractors usually bill a fixed price to their customers at the end
of the month. A limited number of generators try to constantly supply enough
energy to respect the demand. During a typical day, two peak consumption occur
around lunch time and in the evening. These rush hours are difficult to manage for
the grid which must mobilize more resources to satisfy the energy consumed by the
households. Therefore, the utility cost is higher which impacts on their customers
bill. This issue becomes even more complex with the volatility of renewable energy.

The research nowadays turns toward the demand side with an intelligent way to
influence the loads. Indeed, the trend tends to the reduction and management of
the loads instead of the construction of new generators. Demand side management
allows to optimize the human comfort and health in the building while using the
grid efficiently and inexpensively. 

Demand Response(DR) is the strategy of interest for this project. This involves the installation of a more
enhance equipment such as smart meters. These devices allow a two-ways/realtime
communication between the grid and the households to transmit the requests,
household consumption or electricity tariffs. Many DR program are ongoing on the
European scale and in the US. Significant economic savings were proven thanks
to peak demand flattening that reduces the energy costs. Lack of interoperability
between smart grid elements and technological immaturity of certain smart grid
components represent the most common technical obstacle for the DR technologies.
This study assumes that the DR equipment is available in the household. Electricity
tariffs forecast is transmitted from the grid one day ahead. According to this prediction,
a price based DR strategy is carried out for one single building. For a large
scale, if many households consume energy when the price is low, the DR purpose is
achieved and the power peak consumption flatten.
This project aims to maximize the temperature comfort in the building using a
Model Predictive Control (MPC) technique while minimizing the electricity bill.
Batteries and PV panels are accounted in the energy management strategy.
